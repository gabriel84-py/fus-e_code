import board
import busio
import digitalio
import adafruit_bmp3xx
import time
import adafruit_gps
from adafruit_lsm6ds import AccelRange, Rate
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32


class Sensors:
    def __init__(self):
        # I2C Fast Mode (400 kHz) : BMP388 et LSM6DSO32 le supportent tous
        # les deux, et chaque property fait sa propre transaction I2C
        self.i2c = busio.I2C(board.GP15, board.GP14, frequency=400000)
        self.baro_setup()
        self.imu_setup()
        self.gps_setup()

    def baro_setup(self):
        self.bmp = adafruit_bmp3xx.BMP3XX_I2C(self.i2c)
        # BMP3XX.__init__ appelle self.reset(), qui remet TOUS les registres
        # par defaut : OSR x1 en pression et en temperature. Sans les trois
        # lignes ci-dessous le capteur vole a OSx1 (sigma ~ 0.40 m) alors que
        # kalman.r = 0.17^2 est calibre pour OSx8 -> r sous-estime d'un
        # facteur 5.5 et toute la caracterisation de bruit du memoire decrit
        # une configuration dans laquelle la fusee ne vole pas.
        self.bmp.pressure_oversampling = 8
        self.bmp.temperature_oversampling = 1
        # IIR laisse a 0 : le filtre du capteur ajouterait son propre retard
        # et invaliderait le sigma mesure au banc. C'est le Kalman qui filtre.
        self.bmp.filter_coefficient = 0
        return self.bmp

    def imu_setup(self):
        self.imu = LSM6DSO32(self.i2c)
        # le driver force RANGE_8G a l'init, soit 78.5 m/s2, pour un pic de
        # force specifique simule a 71.6 m/s2 (7.30 g) : 10 % de marge, que
        # les transitoires d'allumage et la vibration structurelle mangent.
        self.imu.accelerometer_range = AccelRange.RANGE_16G
        # ODR 208 Hz pour un echantillonnage a 35 Hz : reduit le repliement
        # de la vibration dans la bande utile
        self.imu.accelerometer_data_rate = Rate.RATE_208_HZ
        self.imu.gyro_data_rate = Rate.RATE_208_HZ
        return self.imu

    def gps_setup(self):
        RX = board.GP13
        TX = board.GP12
        # 0.2 s : assez pour terminer une trame deja commencee (83 ms a 9600
        # bauds) sans jamais figer la boucle une seconde entiere. Plus court
        # tronquerait les trames et casserait le parsing.
        uart = busio.UART(TX, RX, baudrate=9600, timeout=0.2,
                          receiver_buffer_size=256)
        self.gps = adafruit_gps.GPS(uart, debug=False)
        self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.gps.send_command(b'PMTK220,1000')
        self.last_print = time.monotonic()
        return self.gps, self.last_print

    @property
    def baro_pt(self):
        """Pression (hPa) ET temperature (degC) en UNE seule conversion.

        adafruit_bmp3xx declenche une conversion forcee complete a chaque
        appel de _read(), et les properties .pressure et .temperature
        l'appellent chacune separement : lire les deux coutait deux
        conversions par tour de boucle, soit ~38 ms a OSx8.
        """
        p, t = self.bmp._read()
        return p / 100.0, t

    @property
    def baro_pa(self):
        return self.bmp.pressure

    @property
    def baro_alt(self):
        # redeclenche sa propre lecture I2C de la pression en interne ; si
        # baro_pt a deja ete lu ce tour-ci, preferer recalculer l'altitude
        # a partir de cette valeur (cf compute_altitude() dans main.py)
        return self.bmp.altitude

    @property
    def baro_temp(self):
        return self.bmp.temperature

    @property
    def imu_accel(self):
        # le capteur est oriente de telle maniere que l'axe z de la fusee est
        # aligne en sens et direction avec l'axe x de l'IMU. L'indice [2] du
        # tuple retourne est donc l'axe LONGITUDINAL de la fusee.
        self.accel_x, self.accel_y, self.accel_z = self.imu.acceleration
        return self.accel_z, self.accel_y, self.accel_x

    @property
    def imu_gyro(self):
        self.gyro_x, self.gyro_y, self.gyro_z = self.imu.gyro
        return self.gyro_x, self.gyro_y, self.gyro_z

    @property
    def gps_data(self):
        self.gps.update()
        if not self.gps.has_fix:
            return None
        lat = self.gps.latitude
        lon = self.gps.longitude
        if lat is None or lon is None:
            return None
        alt = self.gps.altitude_m if self.gps.altitude_m is not None else 0.0
        sat = self.gps.satellites if self.gps.satellites is not None else 0
        return lat, lon, alt, sat