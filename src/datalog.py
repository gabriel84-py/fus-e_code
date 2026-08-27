import board
import busio
import digitalio
import time
import storage
import adafruit_sdcard
from sensors import Sensors
from battery import Battery

#timestamp_ms, phase, ax, ay, az, gx, gy, gz, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v

class Datalog:
    def __init__(self):
        # Setup SPI
        spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
        # Setup SDCard
        cs_sd = digitalio.DigitalInOut(board.GP1)
        self.sdcard = adafruit_sdcard.SDCard(spi, cs_sd)
        vfs = storage.VfsFat(self.sdcard)
        storage.mount(vfs, "/sd")
        #initialiser sensors
        self.sensors = Sensors()
        #initialiser battery
        self.batt = Battery()
        self.filename = time.time()
        self.write_to_sdcard("timestamp_ms, phase, ax, ay, az, gx, gy, gz, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v", f"data_{self.filename}.csv")

    def write_to_sdcard(something, filename):
        with open(filename, "w") as f:
            f.write(something + "\r\n")

    def gather_information(self):
        self.timestamp_ms = time.time()
        self.phase = None
        self.ax, self.ay, self.az = self.sensors.imu_accel[0], self.sensors.imu_accel[1], self.sensors.imu_accel[2]
        self. gx, self.gy, self.gz = self.sensors.imu_gyro[0], self.sensors.imu_gyro[1], self.sensors.imu_gyro[2]
        self.pression_pa = self.sensors.baro_pa
        self.temp_c = self.sensors.baro_temp
        self.alt_baro_m = self.sensors.baro_alt
        self.lat, self.lon = self.sensors.gps_latlong[0], self.sensors.gps_latlong[1]
        self.alt_gps_m = self.sensors.gps_alt
        self.z_kalman_m = None
        self.vz_kalman_ms = None
        self.batt_v = self.batt.tension

    def log(self):
        self.write_to_sdcard(f"{self.timestamp_ms}, {self.phase}, {self.ax}, {self.ay}, {self.az}, {self.gx}, {self.gy}, {self.gz}, {self.pression_pa}, {self.temp_c}, {self.alt_baro_m}, {self.lat}, {self.lon}, {self.alt_gps_m}, {self.z_kalman_m,} {self.vz_kalman_ms}, {self.batt_v}", f"data_{self.filename}.csv")


    def send(self):
        pass