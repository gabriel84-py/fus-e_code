class ApogeeDetection:
    def __init__(self):
        self.seuil = 3*0.17 #3 x sigma_h
        self.maxi = 0

    def detection(self, kh_m):
        if kh_m >= self.maxi:
            self.maxi = kh_m
            return False
        else:
            return (self.maxi - kh_m) >= self.seuil