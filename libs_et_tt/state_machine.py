import time
import libs_et_tt.apogee as apogee


class StateMachine:
    """
    PRE_LAUNCH -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED

    Toutes les constantes de temps sont comptees depuis t_decollage.
    Convention capteur : accel est la FORCE SPECIFIQUE lue sur l'axe
    longitudinal (+9.81 au repos, 0 en chute libre), pas l'acceleration
    cinematique. Tout seuil issu d'une simulation OpenRocket doit donc etre
    derive de la colonne "Acceleration verticale" + 9.81.
    """

    # decollage : N echantillons consecutifs au-dessus du seuil.
    # Avec un seul echantillon, un choc de 2 g au pas de tir (pose sur le
    # rail, insertion de l'inflammateur, rafale) faisait derouler toute la
    # machine d'etat en 13 s et la laissait en LANDED avant le vrai
    # decollage : vol entier perdu. Le vrai decollage fournit 77 echantillons
    # consecutifs au-dessus du seuil, la marge est enorme.
    N_BOOST_CONFIRM = 3

    # garde-fou supplementaire : si l'altitude n'a pas grimpe apres
    # T_CONFIRM_S, c'etait un faux depart, on rearme.
    ALT_CONFIRM_M = 15.0
    T_CONFIRM_S = 1.5

    T_BOOST_MAX = 5.0

    # timer de secours d'apogee. A 8.7 s il gagnait sur la detection
    # barometrique des que l'apogee reelle depassait ~300 m, soit 0.2 s de
    # marge sur la simulation a 289 m : la mesure barometrique, qui est tout
    # l'objet du projet, n'aurait jamais eu lieu. Aucune charge pyro n'etant
    # commandee par la carte, ce timer ne sert qu'a etiqueter l'etat : il doit
    # etre le plus tardif possible. Valide jusqu'a 434 m d'apogee.
    T_APOGEE_BACKUP = 12.0

    # atterrissage. Le critere |v| < 0.5 m/s ne peut pas se declencher : une
    # fois la fusee couchee, l'axe longitudinal lit ~0 au lieu de +9.81 et le
    # filtre croit etre en chute libre (vitesse mesuree a -21 m/s en moyenne,
    # 0 % des echantillons sous 0.5 m/s). On utilise la stabilite de
    # l'altitude BAROMETRIQUE BRUTE, qui elle est juste.
    T_LANDED_MAX = 90.0     # 50 s etait plus court que la descente sous un
                            # parachute a 5 m/s depuis 257 m (51 s)
    LANDED_BAND_M = 2.5
    LANDED_HOLD_S = 5.0

    def __init__(self):
        self.apogeedetec = apogee.ApogeeDetection()
        self.state_list = ["PRE_LAUNCH", "BOOST", "COAST", "APOGEE",
                           "DESCENT", "LANDED"]
        self.state = self.state_list[0]

        self.accelLimitForBoost = 19.6    # m/s2 lus, soit 2 g
        # la force specifique ne descend jamais sous -3.4 m/s2 entre le
        # decollage et l'apogee (trainee seule) : l'ancien seuil de -12 n'etait
        # jamais franchi et BOOST->COAST se faisait toujours par le timer 5 s,
        # soit 2.3 s apres le vrai burnout. 0.0 est franchi des la fin de
        # poussee et reste loin des +19.6 de la phase propulsee.
        self.accelLimitForBurnOut = 0.0

        self.t_decollage = 0.0  # !!!! en SECONDES !!!!
        self.h_decollage = 0.0
        self.n_boost = 0        # lu par main.py pour geler le recalage du biais
        self.h_ref_landed = None
        self.t_landed_start = 0.0

    def update(self, t, accel, kalman_speed, kh_m, baro_alt=None):
        # baro_alt : altitude barometrique BRUTE, utilisee seulement pour la
        # detection d'atterrissage. Optionnel pour rester compatible avec les
        # anciens appels a 4 arguments.
        h_land = kh_m if baro_alt is None else baro_alt
        s = self.state

        if s == "PRE_LAUNCH":
            if accel >= self.accelLimitForBoost:
                self.n_boost += 1
            else:
                self.n_boost = 0
            if self.n_boost >= self.N_BOOST_CONFIRM:
                self.state = "BOOST"
                self.t_decollage = t
                self.h_decollage = kh_m

        elif s == "BOOST":
            dt = t - self.t_decollage
            if dt > self.T_CONFIRM_S and (kh_m - self.h_decollage) < self.ALT_CONFIRM_M:
                # faux depart : on rearme completement
                self.state = "PRE_LAUNCH"
                self.n_boost = 0
                self.apogeedetec.reset()
            elif accel <= self.accelLimitForBurnOut or dt >= self.T_BOOST_MAX:
                self.state = "COAST"

        elif s == "COAST":
            if self.apogeedetec.detection(kh_m):
                self.state = "APOGEE"
            elif (t - self.t_decollage) >= self.T_APOGEE_BACKUP:
                self.state = "APOGEEtimer"

        elif s == "APOGEE" or s == "APOGEEtimer":
            self.state = "DESCENT"
            self.h_ref_landed = None

        elif s == "DESCENT":
            if (t - self.t_decollage) >= self.T_LANDED_MAX:
                self.state = "LANDED"
            elif (self.h_ref_landed is None
                  or abs(h_land - self.h_ref_landed) > self.LANDED_BAND_M):
                # l'altitude bouge encore : on redemarre la fenetre
                self.h_ref_landed = h_land
                self.t_landed_start = t
            elif (t - self.t_landed_start) >= self.LANDED_HOLD_S:
                self.state = "LANDED"