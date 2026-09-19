import time

# calculer dt dans main a chaque fois qu'on appelle kalman
# dt en secondes !!!


class Kalman:
    # Rejet d'une correction barometrique dont l'innovation est aberrante.
    # Une seule lecture corrompue (erreur I2C sous vibration) suffisait a
    # deplacer h de plusieurs dizaines de metres, donc a fausser le maximum
    # memorise par ApogeeDetection et a declencher un apogee jusqu'a 1.6 s
    # trop tot. 30 m est tres au-dessus de toute innovation physique : en vol
    # l'ecart prediction/mesure reste sous le metre, meme apres un blocage de
    # la boucle de 0.3 s.
    GATE_M = 30.0

    def __init__(self, dt, Sensors):
        self.sensors = Sensors  # calibrate() en a besoin

        self.b = None
        self.v = 0
        self.h = 0
        self.r = 0.17**2  # calibre sur le bruit BMP388 mesure a OSx8 :
                          # sensors.baro_setup() DOIT programmer OSx8, sinon
                          # le capteur tourne a OSx1 (sigma ~ 0.40 m) et r est
                          # sous-estime d'un facteur 5.5
        self.p_hh = 0.0
        self.p_hv = 0.0
        self.p_hb = 0.0
        self.p_vv = 0.0
        self.p_vb = 0.0
        self.p_bb = 1.0

        self.n_rejets = 0  # nb de corrections baro rejetees, a surveiller

        # sigma_a ne modelise PAS le bruit electronique de l'accelerometre
        # (2.158e-3 m/s2, datasheet) mais l'erreur du MODELE. Le modele suppose
        # que l'axe longitudinal reste vertical : des l'ejection la fusee
        # s'incline et l'axe ne lit plus que g*cos(theta), soit une
        # acceleration fantome de g*(1-cos theta) que le filtre integre deux
        # fois. Avec la valeur datasheet, p_hh s'effondre a 1.5e-3 m2, le gain
        # tombe a 0.05 et le filtre cesse d'ecouter le barometre : erreur
        # reelle de 12 m en descente pour un sigma annonce de 4.5 cm (274
        # sigma). Effets mesures en passant a 0.05 : biais systematique sur
        # l'apogee mesuree -0.50 m -> -0.01 m, erreur en descente 12.3 m ->
        # 0.8 m, et plus de declenchement premature sous vibration.
        self.sigma_a = 0.05

        # q_b : le biais accelerometrique est recale en continu par main.py
        # tant qu'on est en PRE_LAUNCH (moyenne glissante sur la mesure au
        # repos), donc le filtre n'a pas besoin de le reestimer lui-meme.
        self.q_b = 1e-9

    def calibrate(self, n_samples=300):
        # sert de valeur initiale seulement : main.py continue d'affiner b
        # pendant toute la phase PRE_LAUNCH
        samples = []
        for _ in range(n_samples):
            samples.append(self.sensors.imu_accel[2])

        mean = sum(samples) / n_samples
        variance = sum((a - mean) ** 2 for a in samples) / (n_samples - 1)

        self.b = mean  # pas la moyenne des ecarts, ca vaut tjrs 0 par def
        self.p_bb = variance / n_samples

        return mean, variance

    def prediction(self, dt, a_meas):
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

    def update(self, baro_alt):
        # garde NaN + rejet des innovations aberrantes : on conserve la
        # prediction plutot que d'injecter une mesure fausse
        if baro_alt != baro_alt or abs(baro_alt - self.h) > self.GATE_M:
            self.n_rejets += 1
            return self.h, self.v, self.b

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
        new_p_vv = self.p_vv - k_v * self.p_hv  # p_hv, pas p_vv
        new_p_vb = self.p_vb - k_v * self.p_hb
        new_p_bb = self.p_bb - k_b * self.p_hb

        self.p_hh = new_p_hh
        self.p_hv = new_p_hv
        self.p_hb = new_p_hb
        self.p_vv = new_p_vv
        self.p_vb = new_p_vb
        self.p_bb = new_p_bb

        return self.h, self.v, self.b