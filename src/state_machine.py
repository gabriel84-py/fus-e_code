import time

class StateMachine:
    #mettre le gps a ON dès que la phase decente est activée
    def __init__(self):
        self.state_list = ["PRE_LAUNCH", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"]
        self.state = self.state_list[0]
        self.accelLimitForBoost = 19.6 #en m/s^2
        self.accelLimitForBurnOut = -12 #en m/s^2
        self.t_decollage = 0.0 # !!!! en SECONDES !!!!
        self.ncons = 0

    def update(self, t, accel, kalman_speed, apogee_atteinte): #!!!!!! kalman_speeden m/s; t en SECONDES avec time.monotonic() !!!!!!!

        #limit for boost
        if self.state == self.state_list[0]:
            if accel >= self.accelLimitForBoost:
                self.state = self.state_list[1]
                self.t_decollage = t

        #limit for COAST
        elif self.state == self.state_list[1]:
            if accel <= self.accelLimitForBurnOut:
                self.state = self.state_list[2]
            elif (t - self.t_decollage) >= 5:
                self.state = self.state_list[2]

        #limit for Apogee
        elif self.state == self.state_list[2]:
            if apogee_atteinte:
                self.state = self.state_list[3]
            elif (t - self.t_decollage) >= 8.7:
                self.state = "APOGEEtimer"

        #limit for DESCENT
        elif (self.state == self.state_list[3]) or (self.state == "APOGEEtimer"):
            self.state = self.state_list[4]

        #limit for LANDED
        elif self.state == self.state_list[4]:
            if (t - self.t_decollage) >= 50:
                self.state = self.state_list[5]
            if abs(kalman_speed) < 0.5:
                self.ncons += 1
            else:
                self.ncons = 0
            if self.ncons == 200:
                self.state = self.state_list[5]
