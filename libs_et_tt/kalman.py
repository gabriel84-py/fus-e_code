import time

#calculer dt dans main a chaque fois qu'on appelle kalman
#dt en secondes !!!

class Kalman:
    def __init__(self, dt, Sensors):
        self.sensors = Sensors  #calibrate() en a besoin

        self.b = None
        self.v = 0
        self.h = 0
        self.r = 0.17**2  # calibre sur le bruit BMP388 mesure a l'oversampling actuel (OSx8)
        self.p_hh = 0.0
        self.p_hv = 0.0
        self.p_hb = 0.0
        self.p_vv = 0.0
        self.p_vb = 0.0
        self.p_bb = 1.0

        # sigma_a seul est fixe ; q_v/q_h sont recalcules a chaque prediction()
        # avec le vrai dt mesure, pas avec ce dt d'init (boucle a freq variable)
        self.sigma_a = 2.158e-3
        # q_b : reglage final, valide en 3 etapes sur banc (fusee immobile,
        # print p_hh/p_bb) + verification numerique. 1e-2 -> p_bb bloque a
        # ~0.35-0.4. 1e-3 -> ~0.05. 1e-4 -> ~0.0075, mais p_hh plafonne a
        # ~0.011-0.013 (loin des 0.002163 documentes) : q_b n'etait plus le
        # facteur dominant a ce niveau. Rejoue numeriquement les equations
        # de prediction()/update() ci-dessous avec le vrai rythme mesure
        # (prediction ~33 Hz, correction baro ~4.5 Hz) : confirme p_hh a
        # 0.0100-0.0145 pour q_b=1e-4 (coherent avec la mesure banc), puis
        # sweep -> p_hh=0.00200 pour q_b=1e-9, tres proche des 0.002163
        # documentes. sigma_a (valeur datasheet) n'a pas besoin d'etre
        # corrige : une fois q_b correctement dimensionne, il redonne le
        # bon ordre de grandeur.
        self.q_b = 1e-9

    def calibrate(self, n_samples=300):
        samples = []
        for _ in range(n_samples):
            samples.append(self.sensors.imu_accel[2])

        mean = sum(samples) / n_samples
        variance = sum((a - mean) ** 2 for a in samples) / (n_samples - 1)

        self.b = mean  # pas la moyenne des ecarts, ca vaut tjrs 0 par def de la moyenne
        self.p_bb = variance / n_samples

        return mean, variance

    def prediction(self, dt, a_meas):  # a_meas en param, plus de self.sensors ici
        h, v, b = self.h, self.v, self.b
        self.a_corr = a_meas - self.b

        q_v = self.sigma_a**2 * dt
        q_h = self.sigma_a**2 * dt**3 / 3  # dt^3/3, pas dt^2/2

        self.h = h + (v * dt) + (0.5 * self.a_corr * dt**2)
        self.v = v + (self.a_corr * dt)
        self.b = b

        a11 = self.p_hh + dt * self.p_hv - 0.5 * dt * dt * self.p_hb
        a12 = self.p_hv + dt * self.p_vv - 0.5 * dt * dt * self.p_vb
        a13 = self.p_hb + dt * self.p_vb - 0.5 * dt * dt * self.p_bb
        a22 = self.p_vv - dt * self.p_vb
        a23 = self.p_vb - dt * self.p_bb

        self.p_hh = a11 + dt * a12 - 0.5 * dt * dt * a13 + q_h
        self.p_hv = a12 - dt * a13
        self.p_hb = a13
        self.p_vv = a22 - dt * a23 + q_v
        self.p_vb = a23
        self.p_bb = self.p_bb + self.q_b

        return self.h, self.v, self.b

    def update(self, baro_alt):  # baro_alt en param, plus de self.sensors ici
        innov = baro_alt - self.h

        s = self.p_hh + self.r  # meme s pour les 3 gains
        k_h = self.p_hh / s
        k_v = self.p_hv / s  # deja en m/s, pas besoin de diviser par dt
        k_b = self.p_hb / s  # deja en m/s^2 direct

        self.h += k_h * innov
        self.v += k_v * innov
        self.b += k_b * innov

        # new_p_xx obligatoire : sinon un calcul utilise une valeur deja
        # modifiee au lieu de l'ancienne, resultat faux
        new_p_hh = self.p_hh - k_h * self.p_hh
        new_p_hv = self.p_hv - k_h * self.p_hv
        new_p_hb = self.p_hb - k_h * self.p_hb
        new_p_vv = self.p_vv - k_v * self.p_hv  # p_hv, pas p_vv : correction passe tjrs par h
        new_p_vb = self.p_vb - k_v * self.p_hb
        new_p_bb = self.p_bb - k_b * self.p_hb

        self.p_hh = new_p_hh
        self.p_hv = new_p_hv
        self.p_hb = new_p_hb
        self.p_vv = new_p_vv
        self.p_vb = new_p_vb
        self.p_bb = new_p_bb

        return self.h, self.v, self.b