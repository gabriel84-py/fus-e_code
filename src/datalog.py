import board
import busio
import digitalio
import time
import storage
import adafruit_sdcard
from telemetry import Telemetry

#timestamp_ms, phase, ax, ay, az, gx, gy, gz, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v

class Datalog:
    def __init__(self, radio_cs_pin, radio_reset_pin):
        spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

        cs_sd = digitalio.DigitalInOut(board.GP1)
        self.sdcard = adafruit_sdcard.SDCard(spi, cs_sd)
        vfs = storage.VfsFat(self.sdcard)
        storage.mount(vfs, "/sd")

        self.telem = Telemetry(spi, radio_cs_pin, radio_reset_pin)

        self.filename = f"data_{time.time()}.csv"
        self.write_to_sdcard(
            "timestamp_ms, phase, ax, ay, az, gx, gy, gz, pression_pa, "
            "temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, "
            "vz_kalman_ms, batt_v",
            mode="w",  # fichier neuf, en-tete seulement
        )

    def write_to_sdcard(self, something, mode="a"):
        # "a" par defaut : on ajoute a la suite, on n'ecrase pas le vol
        with open(self.filename, mode) as f:
            f.write(something + "\r\n")

    def _formatter(self, timestamp, phase, accel, gyro, pression_pa,
                    temp_c, alt_baro_m, lat, lon, alt_gps_m,
                    z_kalman_m, vz_kalman_ms, batt_v):
        ax, ay, az = accel
        gx, gy, gz = gyro
        return (
            f"{timestamp}, {phase}, {ax}, {ay}, {az}, {gx}, {gy}, {gz}, "
            f"{pression_pa}, {temp_c}, {alt_baro_m}, {lat}, {lon}, "
            f"{alt_gps_m}, {z_kalman_m}, {vz_kalman_ms}, {batt_v}"
        )

    def log(self, timestamp, phase, accel, gyro, pression_pa, temp_c,
             alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms,
             batt_v):
        ligne = self._formatter(timestamp, phase, accel, gyro, pression_pa,
                                 temp_c, alt_baro_m, lat, lon, alt_gps_m,
                                 z_kalman_m, vz_kalman_ms, batt_v)
        self.write_to_sdcard(ligne)

    def send(self, timestamp, phase, accel, gyro, pression_pa, temp_c,
              alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms,
              batt_v):
        ligne = self._formatter(timestamp, phase, accel, gyro, pression_pa,
                                 temp_c, alt_baro_m, lat, lon, alt_gps_m,
                                 z_kalman_m, vz_kalman_ms, batt_v)
        self.telem.send(ligne)


"""
accel = sensors.imu_accel
gyro = sensors.imu_gyro
gps = sensors.gps_latlong
lat, lon = gps if gps is not None else (None, None)

datalog.log(
    time.monotonic(), sm.state, accel, gyro,
    sensors.baro_pa, sensors.baro_temp, sensors.baro_alt,
    lat, lon, sensors.gps_alt,
    kf.h, kf.v, batt.tension
)
"""