# demo_usb.py - LUNATIK, experiences de l'oral OdPF
#
# Envoie sur l'USB (console serie) une ligne par tour de boucle, dans le
# MEME ORDRE que le log SD de datalog.py :
#   timestamp_s, phase, a_lat1, a_lat2, a_long, gx, gy, gz, pression_hpa,
#   temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v
#
# Pas de SD, pas de radio, pas de machine d'etat : la colonne phase vaut
# toujours "DEMO". Le buzzer ne sert qu'a signaler que la carte est prete,
# qu'un recalage est fini, ou qu'il y a une erreur.
#
# Installation : copier ce fichier sur CIRCUITPY sous le nom code.py.
# CircuitPython lance code.py en priorite sur main.py : supprimer code.py
# suffit pour revenir au firmware de vol.
#
# Garder la carte IMMOBILE, DEBOUT (axe long vertical), pendant les ~5 s
# de calibration du demarrage. Trois bips courts = calibration finie, la
# carte est prete et les trames commencent.
# Trois bips longs = erreur au demarrage : la carte redemarre toute seule.
#
# ---- Commandes recues du PC (une ligne de texte, terminee par Entree) ----
#   r          recalage : biais de l'accelerometre + remise a zero du Kalman
#              (carte immobile ~0,5 s). Reponse "#RECAL ok ..." ou
#              "#RECAL bouge ..." si la carte a bouge pendant la mesure.
#   gps on     rallume la lecture du GPS        -> "#GPS on"
#   gps off    coupe la lecture du GPS          -> "#GPS off"
#   os N       surechantillonnage de la pression, N = 1 2 4 8 16 32
#              -> "#OS N r=..." (r du Kalman ajuste au nouveau bruit)
#   ?          etat de la carte                 -> "#ETAT ..."
# Les lignes qui commencent par "#" n'ont pas 17 colonnes : tous les
# programmes du PC les ignorent quand ils cherchent des mesures.

import math
import sys
import time

import board
import digitalio
import supervisor

import battery
import kalman
import sensors

DT = 0.01                    # dt nominal, seulement pour le constructeur
KALMAN_UPDATE_PERIOD = 0.22  # correction barometrique, comme en vol
GROUND_PRESSURE_SAMPLES = 50

# Recalage continu du biais accelerometrique (EMA du firmware de vol).
# False pour la demo de derive : sinon la derive de l'accelerometre est effacee.
RECALAGE_BIAIS = False
ALPHA_BIAIS = 0.02

# GPS coupe par defaut : inutile en interieur, et chaque lecture d'une phrase
# NMEA en cours d'arrivee bloque la boucle jusqu'a ~70 ms, deux fois par
# seconde, ce qui rend la cadence irreguliere. Commande "gps on" pour le
# rallumer sans redemarrer la carte.
GPS_ACTIF = False
# Quand il est actif, le GPS n'est lu qu'un tour sur GPS_TOURS. Chaque
# gps.update() ne lit qu'une phrase NMEA : il faut l'appeler au moins ~3 fois
# par seconde, sinon le buffer UART de 256 octets deborde.
GPS_TOURS = 10

# Recalage a la demande (commande "r")
N_RECAL = 300           # mesures de l'accelerometre (~0,4 s)
N_RECAL_BARO = 5        # mesures du barometre pour le zero du Kalman
SEUIL_IMMOBILE = 0.15   # m/s2 : ecart-type au-dela duquel la carte a bouge

# Modele de bruit du barometre mesure au banc (memoire, partie 2) :
# sigma^2 = SIGMA_OS1^2 / OS + PLANCHER^2. Sert a ajuster r du Kalman
# quand on change le surechantillonnage en direct.
SIGMA_OS1 = 0.41        # m, bruit a x1
PLANCHER = 0.0885       # m, plancher de bruit
OS_VALIDES = (1, 2, 4, 8, 16, 32)

PHASE = "DEMO"

# Bip "carte prete" : BIP_PRET_N bips de BIP_PRET_S secondes.
# BIP_PRET = False pour une carte totalement silencieuse.
BIP_PRET = True
BIP_PRET_N = 3
BIP_PRET_S = 0.08

# Le PC ne doit jamais pouvoir faire redemarrer code.py en ecrivant sur le
# lecteur CIRCUITPY pendant une demo : on coupe le rechargement automatique.
try:
    supervisor.runtime.autoreload = False
except AttributeError:
    try:
        supervisor.disable_autoreload()
    except AttributeError:
        pass

# console propre : pas de sequences d'echappement de la barre de statut
try:
    supervisor.status_bar.console = False
except AttributeError:
    pass

# buzzer force a 0 : une broche laissee flottante peut faire gresiller
buzzer = digitalio.DigitalInOut(board.GP3)
buzzer.direction = digitalio.Direction.OUTPUT
buzzer.value = False


def bip(n, duree):
    # bloquant : appele seulement hors des mesures (demarrage, recalage)
    for i in range(n):
        buzzer.value = True
        time.sleep(duree)
        buzzer.value = False
        if i < n - 1:
            time.sleep(duree)


def compute_altitude(pressure_hpa, sea_level_hpa):
    return 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** 0.1903)


def r_pour(os_n):
    """Variance du bruit barometrique attendue pour un surechantillonnage."""
    return SIGMA_OS1 ** 2 / os_n + PLANCHER ** 2


# ---- init --------------------------------------------------------------
# Une erreur ici (capteur absent, bus I2C bloque) ne doit pas laisser la
# carte muette et figee : trois bips longs, puis on relance code.py.
try:
    sen = sensors.Sensors()
    bat = battery.Battery()
    kal = kalman.Kalman(DT, sen)
    kal.calibrate(300)

    samples = []
    for _ in range(GROUND_PRESSURE_SAMPLES):
        samples.append(sen.baro_pt[0])
        time.sleep(0.05)
    sea_level = sum(samples) / GROUND_PRESSURE_SAMPLES
except Exception as e:
    print("#ERREUR init :", e)
    bip(3, 0.5)
    time.sleep(2)
    supervisor.reload()

os_actuel = 8                # programme dans sensors.baro_setup()

print("timestamp_s, phase, a_lat1, a_lat2, a_long, gx, gy, gz, "
      "pression_hpa, temp_c, alt_baro_m, lat, lon, alt_gps_m, "
      "z_kalman_m, vz_kalman_ms, batt_v")

# calibration finie : on previent avant la premiere trame
if BIP_PRET:
    bip(BIP_PRET_N, BIP_PRET_S)

# valeurs de repli si une lecture echoue
baro_hpa, baro_temp, baro_alt = sea_level, 15.0, 0.0
imu_accel = (0.0, 0.0, 9.81)
gyro = (0.0, 0.0, 0.0)
lat, lon, alt_gps = 0.0, 0.0, 0.0
batv = 0.0

# Horloge en nanosecondes (entiers exacts). time.monotonic() renvoie un
# flottant CircuitPython dont la precision se degrade quand la carte reste
# allumee longtemps : dt et horodatage sont calcules en entiers.
# t0_ns pris APRES les bips : sinon le premier dt contiendrait leur duree.
t0_ns = time.monotonic_ns()
t_prev_pred_ns = t0_ns
t_prev_upd_ns = t0_ns
UPD_NS = int(KALMAN_UPDATE_PERIOD * 1e9)
tour = 0
commande = ""


def horodatage(t_ns):
    """Secondes depuis le lancement, ecrites exactement (6 decimales)."""
    us = (t_ns - t0_ns) // 1000
    return "%d.%06d" % (us // 1000000, us % 1000000)


def recalage():
    """Commande "r" : meme logique que PRE_LAUNCH en vol.

    La carte doit etre immobile, dans l'orientation de la demo. On mesure le
    biais de l'axe long, on remet h au barometre et v a 0, et on repart avec
    des covariances propres. Le PC attend la reponse avant d'armer.
    """
    global t_prev_pred_ns, t_prev_upd_ns
    valeurs = []
    for _ in range(N_RECAL):
        try:
            valeurs.append(sen.imu_accel[2])
        except Exception:
            pass
    if len(valeurs) < N_RECAL // 2:
        print("#RECAL erreur lecture accelerometre")
        return
    n = len(valeurs)
    moy = sum(valeurs) / n
    ecart = math.sqrt(sum((a - moy) ** 2 for a in valeurs) / (n - 1))
    if ecart > SEUIL_IMMOBILE:
        print("#RECAL bouge ecart_type=%.3f" % ecart)
        bip(1, 0.4)
        return
    alts = []
    for _ in range(N_RECAL_BARO):
        try:
            alts.append(compute_altitude(sen.baro_pt[0], sea_level))
        except Exception:
            pass
    h0 = sum(alts) / len(alts) if alts else kal.h
    kal.b = moy
    kal.h = h0
    kal.v = 0.0
    kal.p_hh = kal.r / max(1, len(alts))
    kal.p_hv = 0.0
    kal.p_hb = 0.0
    kal.p_vv = 0.0
    kal.p_vb = 0.0
    kal.p_bb = ecart ** 2 / n
    kal.n_rejets = 0
    t_prev_pred_ns = time.monotonic_ns()
    t_prev_upd_ns = t_prev_pred_ns
    print("#RECAL ok b=%.4f ecart_type=%.4f h=%.3f" % (moy, ecart, h0))
    bip(1, 0.08)


def executer(ligne):
    """Applique une commande recue du PC."""
    global GPS_ACTIF, os_actuel
    mots = ligne.strip().lower().split()
    if not mots:
        return
    if mots[0] == "r":
        recalage()
    elif mots[0] == "gps" and len(mots) == 2 and mots[1] in ("on", "off"):
        GPS_ACTIF = mots[1] == "on"
        print("#GPS " + mots[1])
    elif mots[0] == "os" and len(mots) == 2:
        try:
            n = int(mots[1])
        except ValueError:
            n = 0
        if n in OS_VALIDES:
            try:
                sen.bmp.pressure_oversampling = n
                os_actuel = n
                kal.r = r_pour(n)
                print("#OS %d r=%.5f" % (n, kal.r))
            except Exception as e:
                print("#OS erreur", e)
        else:
            print("#OS refuse : 1 2 4 8 16 32")
    elif mots[0] == "?":
        print("#ETAT gps=%d os=%d recalage_biais=%d b=%.4f rejets=%d" %
              (GPS_ACTIF, os_actuel, RECALAGE_BIAIS, kal.b, kal.n_rejets))
    else:
        print("#INCONNU " + ligne.strip())


def lire_commandes():
    """Lit sans bloquer ce que le PC a envoye ; une commande par ligne."""
    global commande
    while supervisor.runtime.serial_bytes_available:
        c = sys.stdin.read(1)
        if c in "\r\n":
            if commande:
                executer(commande)
            commande = ""
        elif len(commande) < 32:
            commande += c


# ---- boucle --------------------------------------------------------------
while True:
    try:
        lire_commandes()
    except Exception as e:
        commande = ""
        print("#ERREUR commande :", e)

    t_ns = time.monotonic_ns()

    baro_ok = True
    try:
        baro_hpa, baro_temp = sen.baro_pt
        baro_alt = compute_altitude(baro_hpa, sea_level)
    except Exception:
        baro_ok = False

    try:
        imu_accel = sen.imu_accel
        gyro = sen.imu_gyro
    except Exception:
        pass

    if tour % GPS_TOURS == 0:
        if GPS_ACTIF:
            try:
                gps = sen.gps_data
                if gps is not None:
                    lat, lon, alt_gps, _ = gps
            except Exception:
                pass
        try:
            batv = bat.tension
        except Exception:
            pass
    tour += 1

    try:
        # Kalman libre : pas de forcage de h et v a 0 comme en PRE_LAUNCH.
        # Le recalage se fait a la demande (commande "r").
        dt = (t_ns - t_prev_pred_ns) / 1e9
        t_prev_pred_ns = t_ns
        kal_h, kal_v, _ = kal.prediction(dt, imu_accel[2])
        if baro_ok and t_ns - t_prev_upd_ns >= UPD_NS:
            kal_h, kal_v, _ = kal.update(baro_alt)
            t_prev_upd_ns = t_ns

        if RECALAGE_BIAIS:
            kal.b += ALPHA_BIAIS * (imu_accel[2] - kal.b)

        a1, a2, a3 = imu_accel
        gx, gy, gz = gyro
        print(f"{horodatage(t_ns)}, {PHASE}, {a1}, {a2}, {a3}, {gx}, {gy}, {gz}, "
              f"{baro_hpa}, {baro_temp}, {baro_alt}, {lat}, {lon}, "
              f"{alt_gps}, {kal_h}, {kal_v}, {batv}")
    except Exception as e:
        # une erreur de calcul ne doit jamais arreter les mesures
        print("#ERREUR boucle :", e)
