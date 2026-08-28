from sensors import Sensors
import time

#calculer dt dans main a chaque fois qu'on appelle kalman

class Kalman:
    def __init__(self):
        #initialiser sensors
        self.sensors = Sensors()
        self.b = None
        self.v = 0
        self.h = 0
        self.r = 17**2 #erreur d'unité, je te laisse chercher !
        self.p = 0
        self.q = None

    def prediction(self, dt):
        self.a_corr = self.sensors.imu_accel[2] - self.b
        self.v = self.v + (self.a_corr * dt)
        self.h = self.h + (self.v * dt) + (0.5 * self.a_corr * dt**2)
        return self.a_corr, self.v, self.h

    def update(self):
        self.innov = self.sensors.baro_alt - self.h
        self.p += self.q
        self.k = self.p /(self.p + self.r)
        self.h = self.h + self.k * self.innov
        self.p = (1 - self.k) * self.p
        return self.h

