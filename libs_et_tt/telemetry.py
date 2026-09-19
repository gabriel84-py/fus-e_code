import digitalio
import adafruit_rfm9x

# RadioHead / adafruit_rfm9x limite la charge utile a 255 octets, moins 4
# octets d'entete (destination, source, id, flags) geres par la librairie.
MAX_PAYLOAD_BYTES = 251

FREQUENCY_MHZ = 868.0
BAUDRATE = 1_000_000

# adafruit_rfm9x initialise xmit_timeout a 2.0 s et send() BOUCLE en attente
# du TX_DONE jusqu'a ce timeout. Antenne debranchee, conflit SPI avec la SD,
# module qui decroche : 2 s pendant lesquelles ni le Kalman ni la machine
# d'etat ne tournent. Si ca tombe autour de l'apogee, la mesure est perdue.
# 0.3 s couvre largement les ~180 ms d'airtime mesures.
XMIT_TIMEOUT_S = 0.3


class Telemetry:
    def __init__(self, spi, cs_pin, reset_pin, tx_power=13):
        # spi doit etre le bus SPI deja cree par l'appelant (partage avec la
        # carte microSD, cf section 8.5 du memoire). Ne pas recreer un
        # busio.SPI ici.
        cs = digitalio.DigitalInOut(cs_pin)
        reset = digitalio.DigitalInOut(reset_pin)
        self.rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, FREQUENCY_MHZ,
                                          baudrate=BAUDRATE)
        self.rfm9x.tx_power = tx_power
        self.rfm9x.xmit_timeout = XMIT_TIMEOUT_S
        self.n_echecs = 0

    def send(self, message):
        payload = str(message).encode("utf-8")
        if len(payload) > MAX_PAYLOAD_BYTES:
            payload = payload[:MAX_PAYLOAD_BYTES]
        try:
            ok = bool(self.rfm9x.send(payload))
        except Exception:
            ok = False
        if not ok:
            self.n_echecs += 1
        return ok