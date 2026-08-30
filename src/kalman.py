from sensors import Sensors
import time

#calculer dt dans main a chaque fois qu'on appelle kalman
#dt en secondes !!!

class Kalman:
    def __init__(self):
        self.b = None
        self.v = 0
        self.h = 0
        self.r = 17**2 #erreur d'unité, je te laisse chercher !
        self.p = 0
        self.q = None

    def calibrate(self):
        pass

    def prediction(self, dt, imu_accel: tuple):
        self.a_corr = imu_accel[2] - self.b
        self.v = self.v + (self.a_corr * dt)
        self.h = self.h + (self.v * dt) + (0.5 * self.a_corr * dt**2)
        return self.a_corr, self.v, self.h

    def update(self, baro_alt: int):
        self.innov = baro_alt - self.h
        self.p += self.q
        self.k = self.p /(self.p + self.r)
        self.h = self.h + self.k * self.innov
        self.p = (1 - self.k) * self.p
        return self.h

