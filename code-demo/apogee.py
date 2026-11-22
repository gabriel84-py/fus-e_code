class ApogeeDetection:
    """
    Critere : l'altitude est passee SEUIL metres sous le maximum courant,
    confirme sur N_CONFIRM echantillons consecutifs.

    Le compteur de confirmation existe parce qu'une seule valeur aberrante
    suffisait a declencher un faux apogee dans la version precedente : un pic
    parasite vers le haut faisait bondir self.maxi, et l'echantillon normal
    suivant se retrouvait immediatement sous maxi - seuil. Deux confirmations
    coutent un echantillon de retard (29 ms a 35 Hz) et rendent le detecteur
    insensible aux valeurs isolees.
    """

    def __init__(self, seuil=0.51, n_confirm=2):  # seuil = 3 x sigma_h
        self.seuil = seuil
        self.n_confirm = n_confirm
        self.maxi = None      # None et pas 0 : une altitude initiale negative
                              # (derive baro au sol) ne doit pas servir de max
        self.n_below = 0

    def reset(self):
        self.maxi = None
        self.n_below = 0

    def detection(self, kh_m):
        # garde NaN : sans elle, un NaN se propage dans self.maxi et les deux
        # comparaisons renvoient False pour toujours -> apogee jamais detecte
        if kh_m != kh_m:
            return False

        if self.maxi is None or kh_m >= self.maxi:
            self.maxi = kh_m
            self.n_below = 0
            return False

        if (self.maxi - kh_m) >= self.seuil:
            self.n_below += 1
        else:
            self.n_below = 0

        return self.n_below >= self.n_confirm