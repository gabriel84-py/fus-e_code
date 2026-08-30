import board
import busio
import digitalio
import time
import storage
import adafruit_sdcard
from sensors import Sensors
from battery import Battery
from telemetry import Telemetry

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
        #initialiser telemetry
        self.telem = Telemetry()
        self.filename = time.time()
        self.write_to_sdcard("timestamp_ms, phase, ax, ay, az, gx, gy, gz, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v", f"data_{self.filename}.csv")

    def write_to_sdcard(self, something, filename):
        with open(filename, "w") as f:
            f.write(something + "\r\n")


    # changer tout metrre en param
    def gather_information(self, timestamp, phase, accel, gyro, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v):
        self.timestamp_ms = timestamp
        self.phase = phase
        self.ax, self.ay, self.az = accel  # accel lu une seule fois par main, plus de pb
        self.gx, self.gy, self.gz = gyro   # pareil pour gyro
        self.pression_pa = pression_pa
        self.temp_c = temp_c
        self.alt_baro_m = alt_baro_m
        self.lat = lat  # plus d'indexation ici, plus de risque de crash si pas de fix
        self.lon = lon
        self.alt_gps_m = alt_gps_m
        self.z_kalman_m = z_kalman_m
        self.vz_kalman_ms = vz_kalman_ms
        self.batt_v = batt_v

    def log(self, timestamp, phase, accel, gyro, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v):
        self.gather_information(timestamp, phase, accel, gyro, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v)
        self.write_to_sdcard(f"{self.timestamp_ms}, {self.phase}, {self.ax}, {self.ay}, {self.az}, {self.gx}, {self.gy}, {self.gz}, {self.pression_pa}, {self.temp_c}, {self.alt_baro_m}, {self.lat}, {self.lon}, {self.alt_gps_m}, {self.z_kalman_m}, {self.vz_kalman_ms}, {self.batt_v}", f"data_{self.filename}.csv")

    def send(self, timestamp, phase, accel, gyro, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v):
        self.gather_information(timestamp, phase, accel, gyro, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v)
        self.telem.send(f"{self.timestamp_ms}, {self.phase}, {self.ax}, {self.ay}, {self.az}, {self.gx}, {self.gy}, {self.gz}, {self.pression_pa}, {self.temp_c}, {self.alt_baro_m}, {self.lat}, {self.lon}, {self.alt_gps_m}, {self.z_kalman_m}, {self.vz_kalman_ms}, {self.batt_v}")


"""
accel = sensors.imu_accel      # lecture unique
gyro = sensors.imu_gyro        # lecture unique
gps = sensors.gps_latlong      # a gerer avant l'appel : None si pas de fix
lat, lon = gps if gps is not None else (None, None)

datalog.gather_information(
    time.monotonic(), sm.state, accel, gyro,
    sensors.baro_pa, sensors.baro_temp, sensors.baro_alt,
    lat, lon, sensors.gps_alt,
    kf.h, kf.v, batt.tension
)
"""