import board
import busio
import digitalio
import adafruit_bmp3xx
import time
import adafruit_gps
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
        return self.bmp

    def imu_setup(self):
        self.imu = LSM6DSO32(self.i2c)
        return self.imu

    def gps_setup(self):
        RX = board.GP13
        TX = board.GP12
        # timeout 1 s (pas 30 s) : au 1 Hz configure par PMTK220,1000, une
        # trame arrive au moins 1x/s en fonctionnement normal ; 30 s aurait
        # pu geler toute la boucle de vol en cas de souci GPS
        uart = busio.UART(TX, RX, baudrate=9600, timeout=1)
        self.gps = adafruit_gps.GPS(uart, debug=False)
        self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.gps.send_command(b'PMTK220,1000')
        self.last_print = time.monotonic()
        return self.gps, self.last_print

    @property
    def baro_pa(self):
        return self.bmp.pressure

    @property
    def baro_alt(self):
        # redeclenche sa propre lecture I2C de la pression en interne ; si
        # baro_pa a deja ete lu ce tour-ci, preferer recalculer l'altitude
        # a partir de cette valeur (cf compute_altitude() dans main.py)
        return self.bmp.altitude

    @property
    def baro_temp(self):
        return self.bmp.temperature

    @property
    def imu_accel(self):
        # attention, le capteur est orienté de telle manière que l'axe z de la fusée est aligné en sens et direction avec l'axe x de l'IMU
        self.accel_x, self.accel_y, self.accel_z = self.imu.acceleration
        return self.accel_z, self.accel_y, self.accel_x #x et z sont donc volontairement intervertis ! je l'ai modif

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
        alt = self.gps.altitude_m if self.gps.altitude_m is not None else 0.0
        sat = self.gps.satellites if self.gps.satellites is not None else 0
        return lat, lon, alt, sat