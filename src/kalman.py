from sensors import Sensors
import time

#calculer dt dans main a chaque fois qu'on appelle kalman
#dt en secondes !!!

class Kalman:
    def __init__(self, dt):
        self.b = None
        self.v = 0
        self.h = 0
        self.r = 0.17**2 #cest bon mtn
        self.p_hh = 0.0
        self.p_hv = 0.0
        self.p_hb = 0.0
        self.p_vv = 0.0
        self.p_vb = 0.0
        self.p_bb = 1.0

        sigma_a = 2.158e-3
        self.q_v = sigma_a**2 * dt
        #self.q_h = sigma_a**2 * dt**3 / 3
        self.q_h = sigma_a**2 * dt**2 / 2 #en intégrant qv c'est ce qu'on a trouvé, t'as une explication pour ta formule ?
        self.q_b = 1 * 10**(-9)

    def calibrate(self, n_samples=300):
        samples = []
        for _ in range(n_samples):
            samples.append(self.sensors.imu_accel[2])

        mean = sum(samples) / n_samples
        variance = sum((a - mean) ** 2 for a in samples) / (n_samples - 1)

        #self.b = mean #mais ça veut dire que le biais c'est la moyenne des accelerations ?
        self.b = sum((a - mean) for a in samples) / n_samples #ça nous paraissait plus logique d'avoir une sorte d'ecart relatif
        self.p_bb = variance / n_samples

        return mean, variance

    def prediction(self, dt):
        a_meas = self.sensors.imu_accel[2]
        h, v, b = self.h, self.v, self.b
        self.a_corr = a_meas - self.b

        self.b = b
        self.v = v + (self.a_corr * dt)
        self.h = h + (v * dt) + (0.5 * self.a_corr * dt**2)
        #c'est ce que t'as fait juste présenté à notre manière
        
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

    def update(self, dt):
        baro_alt = self.sensors.baro_alt
        diff = baro_alt - self.h

        s = self.p_hh + self.r # t'es sur qu'il faut toujours diviser par self.p_hh ...

        k_h = self.p_hh / (self.p_hh + self.r)
        k_v = self.p_hv / (self.p_hv + self.r)
        k_b = self.p_hb / (self.p_hb + self.r)

        self.h += k_h * diff
        #self.v += k_v * diff # c'est bizarre de faire m/s + m non ? (avec innov en m)
        #self.b += k_b * diff # et la des m/s² + m ...
        self.v += k_v * (diff / dt) # ça paraît plus logique en mettant a la mm unité
        self.b += k_b * (diff / dt**2) 

        new_p_hh = self.p_hh - k_h * self.p_hh
        new_p_hv = self.p_hv - k_h * self.p_hv
        new_p_hb = self.p_hb - k_h * self.p_hb
        #pourquoi d'un coup tu changes la deuxieme variable ? (j'ai laissé en commenté ce que t'avais mis de base)
        new_p_vv = self.p_vv - k_v * self.p_vv #self.p_hv 
        new_p_vb = self.p_vb - k_v * self.p_vb #self.p_hb
        new_p_bb = self.p_bb - k_b * self.p_bb #self.p_hb

        #si t'avais pas fait exprès avant alors on peut enlever ça
        self.p_hh = new_p_hh
        self.p_hv = new_p_hv
        self.p_hb = new_p_hb
        self.p_vv = new_p_vv
        self.p_vb = new_p_vb
        self.p_bb = new_p_bb

        return self.h, self.v, self.b

"""à mettre dans le main j'imagine ?
kf = Kalman(r=0.17**2, q_h=..., q_v=..., q_b=...)
kf.calibrate()

# dans la boucle de vol
kf.prediction(dt)
if nouvelle_mesure_baro_disponible:
    kf.update()"""
