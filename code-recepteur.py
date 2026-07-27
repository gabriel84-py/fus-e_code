import board
import busio
import digitalio
import adafruit_rfm9x

spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
cs = digitalio.DigitalInOut(board.GP8)
reset = digitalio.DigitalInOut(board.GP9)
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 868.0, baudrate=1000000)

while True:
    paquet = rfm9x.receive(timeout=5.0)
    if paquet is not None:
        texte = paquet.decode("utf-8", "replace")
        print(texte)
