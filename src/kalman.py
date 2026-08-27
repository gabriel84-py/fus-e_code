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

    def prediction(self, dt):
        self.a_corr = self.sensors.imu_accel[2] - self.b
        self.v = self.v + (self.a_corr * dt)
        self.h = self.h + (self.v * dt) + (0.5 * self.a_corr * dt**2)
