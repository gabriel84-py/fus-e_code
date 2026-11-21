# demo_usb.py - LUNATIK, experiences de l'oral OdPF
#
# Envoie sur l'USB (console serie) une ligne par tour de boucle, dans le
# MEME ORDRE que le log SD de datalog.py :
#   timestamp_s, phase, a_lat1, a_lat2, a_long, gx, gy, gz, pression_hpa,
#   temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v
#
# Pas de SD, pas de radio, pas de machine d'etat : la colonne phase vaut
# toujours "DEMO". Le buzzer ne sert qu'a signaler que la carte est prete.
#
# Installation : copier ce fichier sur CIRCUITPY sous le nom code.py.
# CircuitPython lance code.py en priorite sur main.py : supprimer code.py
# suffit pour revenir au firmware de vol.
#
# Garder la carte IMMOBILE, dans l'orientation des demos, pendant les ~5 s
# de calibration du demarrage. Trois bips courts = calibration finie, la
# carte est prete et les trames commencent.

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

# Le GPS n'est lu qu'un tour sur GPS_TOURS. Chaque gps.update() ne lit
# qu'une phrase NMEA : il faut l'appeler au moins ~3 fois par seconde, sinon
# le buffer UART de 256 octets deborde. 10 tours a ~35 Hz = ~3.5 Hz : OK.
GPS_TOURS = 10

PHASE = "DEMO"

# Bip "carte prete" : BIP_PRET_N bips de BIP_PRET_S secondes.
# BIP_PRET = False pour une carte totalement silencieuse.
BIP_PRET = True
BIP_PRET_N = 3
BIP_PRET_S = 0.08

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
    # bloquant, mais appele une seule fois, avant la boucle de mesure
    for i in range(n):
        buzzer.value = True
        time.sleep(duree)
        buzzer.value = False
        if i < n - 1:
            time.sleep(duree)


def compute_altitude(pressure_hpa, sea_level_hpa):
    return 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** 0.1903)


# ---- init --------------------------------------------------------------
sen = sensors.Sensors()
bat = battery.Battery()
kal = kalman.Kalman(DT, sen)
kal.calibrate(300)

samples = []
for _ in range(GROUND_PRESSURE_SAMPLES):
    samples.append(sen.baro_pt[0])
    time.sleep(0.05)
sea_level = sum(samples) / GROUND_PRESSURE_SAMPLES

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

# t_prev_pred pris APRES les bips : sinon le premier dt de la prediction
# contiendrait la duree des bips
t_prev_pred = time.monotonic()
t_prev_upd = t_prev_pred
tour = 0

# ---- boucle --------------------------------------------------------------
while True:
    sec = time.monotonic()

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

    # Kalman libre : pas de forcage de h et v a 0 comme en PRE_LAUNCH
    dt = sec - t_prev_pred
    t_prev_pred = sec
    kal_h, kal_v, _ = kal.prediction(dt, imu_accel[2])
    if baro_ok and sec - t_prev_upd >= KALMAN_UPDATE_PERIOD:
        kal_h, kal_v, _ = kal.update(baro_alt)
        t_prev_upd = sec

    if RECALAGE_BIAIS:
        kal.b += ALPHA_BIAIS * (imu_accel[2] - kal.b)

    a1, a2, a3 = imu_accel
    gx, gy, gz = gyro
    print(f"{sec}, {PHASE}, {a1}, {a2}, {a3}, {gx}, {gy}, {gz}, "
          f"{baro_hpa}, {baro_temp}, {baro_alt}, {lat}, {lon}, "
          f"{alt_gps}, {kal_h}, {kal_v}, {batv}")