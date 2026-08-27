"""
LUNATIK - Banc de caracterisation du bruit du barometre BMP388
Projet : "Une fusee sait-elle qu'elle redescend ?"

Objectif
--------
Mesurer sigma_p (ecart-type du bruit blanc de pression) pour chaque reglage
de surechantillonnage, et en deduire sigma_h, le bruit equivalent en altitude.

Methode
-------
On n'utilise PAS l'ecart-type brut de l'enregistrement : il est domine par la
derive thermique et par les variations reelles de pression de la piece.
On calcule l'ecart-type des differences entre mesures consecutives :

    sigma_p = sigma_diff / sqrt(2)

La difference est un filtre passe-haut qui elimine toute composante lente.
C'est aussi exactement la grandeur dont depend le taux de faux declenchements
de l'algorithme de detection d'apogee.

Validation
----------
Pour du bruit blanc, l'autocorrelation au retard 1 de la serie des DIFFERENCES
vaut exactement -0.5. Si la valeur mesuree s'en ecarte nettement, il reste du
filtrage quelque part (filtre IIR interne non desactive, moyennage cache).

Materiel
--------
Raspberry Pi Pico + BMP388 (I2C) + carte SD (SPI)
Capteur sous une boite en polystyrene PERCEE D'UN PETIT TROU (jamais etanche).
Piece fermee, ventilation coupee, 15 minutes de chauffe avant mesure.

Sorties
-------
- Un fichier CSV brut par configuration dans /sd/
- Un tableau recapitulatif imprime sur la console et ecrit dans /sd/resume.csv
"""

import board
import busio
import digitalio
import storage
import adafruit_sdcard
import adafruit_bmp3xx
import time
import math
import gc
from array import array


# =====================================================================
# PARAMETRES DE LA CAMPAGNE
# =====================================================================

# Configurations a balayer : (surechantillonnage pression, surech. temperature)
CONFIGS = (
    (1, 1),
    (2, 1),
    (4, 1),
    (8, 1),
    (16, 1),
    (32, 1),
)

NB_ECHANTILLONS = 10000   # 10000 -> incertitude sur sigma de 0.7 %
NB_REJETES = 200          # echantillons jetes apres changement de config
TAILLE_BLOC = 500         # taille des tampons RAM, ecriture SD par blocs
CHAUFFE_S = 900           # 15 min d'auto-echauffement avant la 1re config

R_AIR = 287.05            # constante specifique de l'air sec, J/(kg.K)
G = 9.80665               # m/s^2


# =====================================================================
# INITIALISATION DU MATERIEL
# =====================================================================

def setup_i2c():
    # 400 kHz et non 100 kHz par defaut : la cadence atteignable en depend
    return busio.I2C(board.GP15, board.GP14, frequency=400000)


def setup_bmp(i2c):
    bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
    # Filtre IIR interne DESACTIVE : il ajoute un retard de groupe invisible
    # qui fausserait a la fois sigma_p et la mesure du retard de detection.
    bmp.filter_coefficient = 0
    return bmp


def setup_sd(spi):
    cs_sd = digitalio.DigitalInOut(board.GP1)
    sdcard = adafruit_sdcard.SDCard(spi, cs_sd)
    vfs = storage.VfsFat(sdcard)
    storage.mount(vfs, "/sd")
    return sdcard


def desactiver_radio(spi):
    """Maintient le RFM95 en reset pendant toute la campagne.

    Le module radio partage le bus SPI avec la carte SD. Une emission
    pendant l'acquisition perturberait le timing et l'alimentation.
    """
    reset_rfm = digitalio.DigitalInOut(board.GP9)
    reset_rfm.direction = digitalio.Direction.OUTPUT
    reset_rfm.value = False
    cs_rfm = digitalio.DigitalInOut(board.GP8)
    cs_rfm.direction = digitalio.Direction.OUTPUT
    cs_rfm.value = True
    return reset_rfm, cs_rfm


# =====================================================================
# LECTURE BAS NIVEAU
# =====================================================================

def lire_brut(bmp):
    """Retourne (pression_Pa, temperature_C) en UNE seule conversion forcee.

    Passer par bmp.pressure puis bmp.temperature declencherait deux
    conversions successives : cadence divisee par deux, et pression et
    temperature ne correspondraient plus au meme instant.
    """
    try:
        brut = bmp._read()
        p, t = brut[0], brut[1]
        # Garde-fou : selon la version de la bibliotheque, l'ordre ou
        # l'unite peuvent differer. On verifie la plausibilite physique.
        if not (30000.0 < p < 120000.0):
            if 30000.0 < t < 120000.0:
                p, t = t, p
            elif 300.0 < p < 1200.0:
                p = p * 100.0
        return p, t
    except AttributeError:
        # Repli si _read() n'existe pas : deux conversions, cadence divisee
        return bmp.pressure * 100.0, bmp.temperature


# =====================================================================
# ACCUMULATEURS STATISTIQUES EN FLUX
# =====================================================================

class Stats:
    """Statistiques calculees en flux : aucun tableau global en RAM."""

    def __init__(self):
        self.n = 0
        self.somme_p = 0.0
        self.somme_p2 = 0.0
        self.somme_t = 0.0
        self.n_d = 0
        self.somme_d = 0.0
        self.somme_d2 = 0.0
        self.n_dd = 0
        self.somme_dd = 0.0
        self.p_prec = None
        self.d_prec = None
        self.t_debut_us = None
        self.t_fin_us = None

    def ajouter(self, t_us, p, temp):
        if self.t_debut_us is None:
            self.t_debut_us = t_us
        self.t_fin_us = t_us

        self.n += 1
        self.somme_p += p
        self.somme_p2 += p * p
        self.somme_t += temp

        if self.p_prec is not None:
            d = p - self.p_prec
            self.n_d += 1
            self.somme_d += d
            self.somme_d2 += d * d
            if self.d_prec is not None:
                self.n_dd += 1
                self.somme_dd += self.d_prec * d
            self.d_prec = d
        self.p_prec = p

    # --- grandeurs derivees ---

    def moyenne_p(self):
        return self.somme_p / self.n

    def moyenne_t(self):
        return self.somme_t / self.n

    def sigma_naif(self):
        """Ecart-type brut. Surestime le bruit : a montrer pour comparaison."""
        m = self.moyenne_p()
        var = self.somme_p2 / self.n - m * m
        return math.sqrt(var) if var > 0.0 else 0.0

    def sigma_diff(self):
        m = self.somme_d / self.n_d
        var = self.somme_d2 / self.n_d - m * m
        return math.sqrt(var) if var > 0.0 else 0.0

    def sigma_p(self):
        return self.sigma_diff() / math.sqrt(2.0)

    def r1_differences(self):
        """Autocorrelation au retard 1 des differences. Attendu : -0.5."""
        sd = self.sigma_diff()
        if sd <= 0.0 or self.n_dd == 0:
            return float("nan")
        md = self.somme_d / self.n_d
        cov = self.somme_dd / self.n_dd - md * md
        return cov / (sd * sd)

    def cadence_hz(self):
        duree = (self.t_fin_us - self.t_debut_us) / 1e6
        return (self.n - 1) / duree if duree > 0.0 else 0.0

    def gradient_pa_par_m(self):
        """rho * g, calcule depuis la pression et la temperature mesurees."""
        rho = self.moyenne_p() / (R_AIR * (self.moyenne_t() + 273.15))
        return rho * G

    def sigma_h_m(self):
        return self.sigma_p() / self.gradient_pa_par_m()

    def incert_relative_sigma(self):
        """Incertitude relative sur un ecart-type : 1 / sqrt(2 (M - 1))."""
        return 1.0 / math.sqrt(2.0 * (self.n_d - 1)) if self.n_d > 1 else float("nan")


# =====================================================================
# ACQUISITION D'UNE CONFIGURATION
# =====================================================================

def acquerir(bmp, os_p, os_t, nom_fichier):
    print("")
    print("=" * 62)
    print("Configuration : surechantillonnage pression x{}, temperature x{}".format(os_p, os_t))
    print("=" * 62)

    bmp.pressure_oversampling = os_p
    bmp.temperature_oversampling = os_t
    bmp.filter_coefficient = 0

    # Purge : les premieres conversions apres changement de reglage
    # ne sont pas representatives.
    for _ in range(NB_REJETES):
        lire_brut(bmp)

    stats = Stats()

    # Tampons preallouees : aucune allocation dans la boucle critique,
    # donc pas de pause du ramasse-miettes pendant la mesure.
    buf_t = array("I", [0] * TAILLE_BLOC)
    buf_p = array("f", [0.0] * TAILLE_BLOC)
    buf_c = array("f", [0.0] * TAILLE_BLOC)

    with open(nom_fichier, "w") as f:
        f.write("t_us,pression_Pa,temperature_C\r\n")

        t0_ns = time.monotonic_ns()
        restants = NB_ECHANTILLONS

        while restants > 0:
            n_bloc = TAILLE_BLOC if restants >= TAILLE_BLOC else restants
            gc.collect()

            # --- boucle critique : lecture au plus vite, sans temporisation ---
            for i in range(n_bloc):
                p, c = lire_brut(bmp)
                buf_t[i] = (time.monotonic_ns() - t0_ns) // 1000
                buf_p[i] = p
                buf_c[i] = c
            # --- fin de la boucle critique ---

            morceaux = []
            for i in range(n_bloc):
                stats.ajouter(buf_t[i], buf_p[i], buf_c[i])
                morceaux.append("{},{:.4f},{:.4f}".format(buf_t[i], buf_p[i], buf_c[i]))
            f.write("\r\n".join(morceaux))
            f.write("\r\n")

            restants -= n_bloc
            print("  {} / {} echantillons".format(NB_ECHANTILLONS - restants, NB_ECHANTILLONS))

    afficher(stats)
    return stats


def afficher(s):
    print("")
    print("  Cadence reelle atteinte  : {:8.2f} Hz".format(s.cadence_hz()))
    print("  Pression moyenne         : {:8.1f} Pa".format(s.moyenne_p()))
    print("  Temperature moyenne      : {:8.2f} C".format(s.moyenne_t()))
    print("  Gradient rho.g           : {:8.3f} Pa/m".format(s.gradient_pa_par_m()))
    print("")
    print("  sigma NAIF (a ne pas utiliser) : {:8.3f} Pa".format(s.sigma_naif()))
    print("  sigma_diff (differences)       : {:8.3f} Pa".format(s.sigma_diff()))
    print("  sigma_p = sigma_diff / sqrt(2) : {:8.3f} Pa".format(s.sigma_p()))
    print("  sigma_h equivalent             : {:8.2f} cm".format(s.sigma_h_m() * 100.0))
    print("  incertitude relative sur sigma : {:8.2f} %".format(s.incert_relative_sigma() * 100.0))
    print("")
    r1 = s.r1_differences()
    print("  r1 des differences : {:+.4f}   (attendu -0.5000)".format(r1))
    if abs(r1 + 0.5) < 0.05:
        print("  -> bruit blanc confirme, sigma_p est valide")
    else:
        print("  -> ECART : filtrage residuel ou correlation. Verifier que")
        print("     filter_coefficient vaut bien 0 et relire la datasheet.")
    print("")
    print("  Rapport sigma_naif / sigma_p : {:6.2f}".format(
        s.sigma_naif() / s.sigma_p() if s.sigma_p() > 0 else float("nan")))
    print("  (mesure de combien la methode naive surestime le bruit)")


# =====================================================================
# PROGRAMME PRINCIPAL
# =====================================================================

def main():
    i2c = setup_i2c()
    spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
    desactiver_radio(spi)
    setup_sd(spi)
    bmp = setup_bmp(i2c)

    print("Banc de bruit BMP388 - LUNATIK")
    print("Chauffe de {} s avant la premiere mesure...".format(CHAUFFE_S))
    t_debut = time.monotonic()
    while time.monotonic() - t_debut < CHAUFFE_S:
        lire_brut(bmp)
        time.sleep(0.5)
    print("Chauffe terminee.")

    resultats = []
    for os_p, os_t in CONFIGS:
        nom = "/sd/bruit_p{:02d}_t{:02d}.csv".format(os_p, os_t)
        s = acquerir(bmp, os_p, os_t, nom)
        resultats.append((os_p, os_t, s))

    # --- tableau recapitulatif ---
    entete = "os_p,os_t,cadence_Hz,sigma_diff_Pa,sigma_p_Pa,sigma_h_cm,r1_diff,gradient_Pa_par_m,T_moy_C,incert_sigma_pct"
    lignes = [entete]
    for os_p, os_t, s in resultats:
        lignes.append("{},{},{:.2f},{:.4f},{:.4f},{:.2f},{:.4f},{:.3f},{:.2f},{:.2f}".format(
            os_p, os_t, s.cadence_hz(), s.sigma_diff(), s.sigma_p(),
            s.sigma_h_m() * 100.0, s.r1_differences(),
            s.gradient_pa_par_m(), s.moyenne_t(), s.incert_relative_sigma() * 100.0))

    with open("/sd/resume.csv", "w") as f:
        f.write("\r\n".join(lignes))
        f.write("\r\n")

    print("")
    print("=" * 62)
    print("RECAPITULATIF - a recopier dans le memoire")
    print("=" * 62)
    for ligne in lignes:
        print(ligne)
    print("")
    print("Ecrit dans /sd/resume.csv")


main()