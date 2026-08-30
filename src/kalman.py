from sensors import Sensors
import time

#calculer dt dans main a chaque fois qu'on appelle kalman
#dt en secondes !!!

class Kalman:
    def __init__(self, dt):
        self.sensors = Sensors()  #calibrate() en a besoin

        self.b = None
        self.v = 0
        self.h = 0
        self.r = 0.17**2
        self.p_hh = 0.0
        self.p_hv = 0.0
        self.p_hb = 0.0
        self.p_vv = 0.0
        self.p_vb = 0.0
        self.p_bb = 1.0

        sigma_a = 2.158e-3
        self.q_v = sigma_a**2 * dt
        self.q_h = sigma_a**2 * dt**3 / 3  # dt^3/3 pas dt^2/2, ca c'etait q_hv pas q_h
        self.q_b = 1 * 10**(-9)  # a redebattre, notre balayage donnait plutot 1e-2

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

        self.h = h + (v * dt) + (0.5 * self.a_corr * dt**2)
        self.v = v + (self.a_corr * dt)
        self.b = b

        a11 = self.p_hh + dt * self.p_hv - 0.5 * dt * dt * self.p_hb
        a12 = self.p_hv + dt * self.p_vv - 0.5 * dt * dt * self.p_vb
        a13 = self.p_hb + dt * self.p_vb - 0.5 * dt * dt * self.p_bb
        a22 = self.p_vv - dt * self.p_vb
        a23 = self.p_vb - dt * self.p_bb

        self.p_hh = a11 + dt * a12 - 0.5 * dt * dt * a13 + self.q_h
        self.p_hv = a12 - dt * a13
        self.p_hb = a13
        self.p_vv = a22 - dt * a23 + self.q_v
        self.p_vb = a23
        self.p_bb = self.p_bb + self.q_b

        return self.h, self.v, self.b

    def update(self, baro_alt):  # baro_alt en param, plus de self.sensors ici
        innov = baro_alt - self.h

        s = self.p_hh + self.r  # meme s pr les 3 gains, pas un s different chacun
        k_h = self.p_hh / s
        k_v = self.p_hv / s  # dej a la bonne unite, pas besoin de diviser par dt
        k_b = self.p_hb / s  # pareil, dej en m/s^2 direct

        self.h += k_h * innov
        self.v += k_v * innov  # pas /dt, casse l'unite sinon
        self.b += k_b * innov  # pas /dt**2 non plus

        new_p_hh = self.p_hh - k_h * self.p_hh
        new_p_hv = self.p_hv - k_h * self.p_hv
        new_p_hb = self.p_hb - k_h * self.p_hb
        new_p_vv = self.p_vv - k_v * self.p_hv  # p_hv pas p_vv, correction passe tjrs par h
        new_p_vb = self.p_vb - k_v * self.p_hb  # meme logique
        new_p_bb = self.p_bb - k_b * self.p_hb  # meme logique

        # les new_p_xx: obligatoire, sinon un calcul utilise une valeur
        # deja modif au lieu de l'ancienne, resultat faux
        self.p_hh = new_p_hh
        self.p_hv = new_p_hv
        self.p_hb = new_p_hb
        self.p_vv = new_p_vv
        self.p_vb = new_p_vb
        self.p_bb = new_p_bb

        return self.h, self.v, self.b

"""a mettre dans le main j'imagine ?
kf = Kalman(dt)
kf.calibrate()

# dans la boucle de vol, main lit les capteurs et passe les valeurs
a = kf.sensors.imu_accel[2]
kf.prediction(dt, a)
if nouvelle_mesure_baro_disponible:
    kf.update(kf.sensors.baro_alt)"""