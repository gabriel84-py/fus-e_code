import board
import busio
import digitalio
import time
import storage
import adafruit_sdcard
import adafruit_rfm9x

spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
cs = digitalio.DigitalInOut(board.GP8)
reset = digitalio.DigitalInOut(board.GP9)
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 868.0, baudrate=1000000)

# Nom de fichier genere UNE SEULE FOIS au demarrage (pas a chaque paquet,
# sinon on se retrouve avec des centaines de fichiers d'une ligne)
FILENAME = "/sd/flightdata_{}.csv".format(int(time.time()))


def setup_sdcard():
    cs_sd = digitalio.DigitalInOut(board.GP1)
    sdcard = adafruit_sdcard.SDCard(spi, cs_sd)
    vfs = storage.VfsFat(sdcard)
    storage.mount(vfs, "/sd")
    return sdcard


sdcard = setup_sdcard()


def write_to_sdcard(line, filename):
    # Ecriture protegee : si la SD a un probleme, on log l'erreur sur la
    # console et on continue de recevoir. La reception LoRa ne doit jamais
    # dependre de la sante de la carte SD.
    try:
        with open(filename, "a") as f:
            f.write(line + "\r\n")
    except OSError as exc:
        print("Erreur ecriture SD:", exc)


while True:
    paquet = rfm9x.receive(timeout=5.0)
    if paquet is not None:
        # rssi/snr ne sont valides que juste apres un receive() reussi,
        # il faut les lire ICI, pas plus tard
        rssi = rfm9x.last_rssi
        snr = rfm9x.last_snr
        texte = paquet.decode("utf-8", "replace").strip()

        # Prefixe TLM: pour que le dashboard PC distingue une vraie trame
        # d'un print de debug quelconque envoye par erreur sur la console
        ligne = "TLM,{},{},{}".format(texte, rssi, snr)

        write_to_sdcard(ligne, FILENAME)
        print(ligne)