import analogio
from board import *

class Battery:
    def __init__(self):
        self.pin = analogio.AnalogIn(board.GP28) #erreur, je te laisse trouver....

    # V_adc = V_batt R2/(R1+R2) -> V_batt = V_adc (R1 +R2)/R2
    @property
    def tension(self):
        v_adc = self.pin.value
        v_adc = v_adc / 65535 * 3.3 #transformer une valeur 16 bits (lecture analogique) en chiffre
        return round(v_adc * (32/22), 2)

    def arret(self):
        return self.pin.deinit()