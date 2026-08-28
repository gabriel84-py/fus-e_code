import digitalio
import adafruit_rfm9x

# RadioHead / adafruit_rfm9x limite la charge utile a 255 octets, moins 4
# octets d'entete (destination, source, id, flags) geres par la librairie.
MAX_PAYLOAD_BYTES = 251

FREQUENCY_MHZ = 868.0
BAUDRATE = 1_000_000


class Telemetry:
    def __init__(self, spi, cs_pin, reset_pin, tx_power=13):
        # spi doit etre le bus SPI deja cree par l'appelant (partage avec la
        # carte microSD, cf section 8.5 du memoire : le RFM95W et la microSD
        # sont sur le meme bus). Ne pas recreer un busio.SPI ici.
        cs = digitalio.DigitalInOut(cs_pin)
        reset = digitalio.DigitalInOut(reset_pin)
        self.rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, FREQUENCY_MHZ, baudrate=BAUDRATE)
        # A ajuster selon la puissance d'emission declaree pour le concours.
        self.rfm9x.tx_power = tx_power

    def send(self, message):
        payload = str(message).encode("utf-8")
        if len(payload) > MAX_PAYLOAD_BYTES:
            payload = payload[:MAX_PAYLOAD_BYTES]
        try:
            return bool(self.rfm9x.send(payload))
        except RuntimeError:
            return False
