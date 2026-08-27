import board
import busio
import digitalio
import adafruit_rfm9x
import time

class Telemetry:
    def __init__(self):
        # Setup SPI
        spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
        # Setup rfm9x
        cs_port_rfm = board.GP8
        cs_rfm = digitalio.DigitalInOut(cs_port_rfm)
        reset_rfm = digitalio.DigitalInOut(board.GP9)
        self.rfm9x = adafruit_rfm9x.RFM9x(spi, cs_rfm, reset_rfm, 868.0, baudrate=1000000)

    def send(self, something):
        self.rfm9x.send(str(something))