import board
import busio
import digitalio
import adafruit_bmp3xx
import time
import adafruit_gps
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32

class Sensors:
    def __init__(self):
        # Setup I2C
        self.i2c = busio.I2C(board.GP15, board.GP14)
        # Setup SPI
        self.spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
        #vérifie : appelle t on les fonctions setup ? non... ptet il faudrais...""

    def baro_setup(self):
        self.bmp = adafruit_bmp3xx.BMP3XX_I2C(self.i2c)
        return self.bmp

    def imu_setup(self):
        self.imu = LSM6DSO32(self.i2c)
        return self.imu

    def gps_setup(self):
        RX = board.GP13
        TX = board.GP12
        uart = busio.UART(TX, RX, baudrate=9600, timeout=30)
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
        return self.bmp.altitude

    @property
    def baro_temp(self):
        return self.bmp.temperature

    @property
    def imu_accel(self):
        self.accel_x, self.accel_y, self.accel_z = self.imu.acceleration
        return self.accel_x, self.accel_y, self.accel_z

    @property
    def imu_gyro(self):
        self.gyro_x, self.gyro_y, self.gyro_z = self.imu.gyro
        return self.gyro_x, self.gyro_y, self.gyro_z

    @property
    def gps_latlong(self):
        self.gps.update()
        if not self.gps.has_fix:
            return None
        else:
            return self.gps.latitude, self.gps.longitude
        
    @property
    def gps_alt(self):
        self.gps.update()
        if not self.gps.has_fix:
            return None
        else:
            return self.gps.altitude_m if self.gps.altitude_m is not None else 0.0
        
    @property
    def gps_sat(self):
        self.gps.update()
        if not self.gps.has_fix:
            return None
        else:
            return self.gps.satellites if self.gps.satellites is not None else 0