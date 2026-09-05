import board
import busio
import digitalio
import adafruit_rfm9x
import storage
import adafruit_sdcard
import time

filedata = f"/sd/flightdata_{time.time()}"

spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
cs = digitalio.DigitalInOut(board.GP8)
reset = digitalio.DigitalInOut(board.GP9)
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 868.0, baudrate=1000000)

def setup_sdcard():
    cs_sd = digitalio.DigitalInOut(board.GP1)
    sdcard = adafruit_sdcard.SDCard(spi, cs_sd)
    vfs = storage.VfsFat(sdcard)
    storage.mount(vfs, "/sd")
    return sdcard

sdcard = setup_sdcard()

def write_to_sdcard(something, filename, mode="a"):
    # "a" par defaut : on ajoute a la suite, on n'ecrase pas le vol
    with open(filename, mode) as f:
        f.write(something + "\r\n")


while True:
    paquet = rfm9x.receive(timeout=5.0)
    if paquet is not None:
        texte = paquet.decode("utf-8", "replace")
        write_to_sdcard(texte, filedata)
        print(texte)
