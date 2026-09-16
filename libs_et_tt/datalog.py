import board
import digitalio
import time
import storage
import adafruit_sdcard

#timestamp_ms, phase, ax, ay, az, gx, gy, gz, pression_pa, temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v

class Datalog:
    def __init__(self, spi, flush_every=10):
        # flush_every : nb de lignes bufferisees en memoire avant
        # reouverture/fermeture reelle du fichier sur la SD. Chaque
        # ouverture/fermeture coute cher sur SD SPI (~30 ms mesures sur
        # banc) ; grouper N lignes par ecriture divise ce cout par N en
        # moyenne (~3 ms/ligne a flush_every=10). En cas de coupure
        # d'alimentation entre deux flush, on perd au plus (flush_every - 1)
        # lignes les plus recentes -- jamais tout le vol. Voir aussi flush()
        # ci-dessous, declenche a chaque changement de phase pour ne jamais
        # perdre la ligne qui marque BOOST/COAST/APOGEE/DESCENT/LANDED.
        cs_sd = digitalio.DigitalInOut(board.GP1)
        self.sdcard = adafruit_sdcard.SDCard(spi, cs_sd)
        vfs = storage.VfsFat(self.sdcard)
        storage.mount(vfs, "/sd")

        self.filename = f"/sd/data_{time.time()}.csv"
        self.flush_every = flush_every
        self._pending = []
        self._last_phase = None

        self._write_lines_now(
            ["timestamp_ms, phase, ax, ay, az, gx, gy, gz, pression_pa, "
             "temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, "
             "vz_kalman_ms, batt_v"],
            mode="w",  # fichier neuf, en-tete seule
        )

    def _write_lines_now(self, lines, mode="a"):
        with open(self.filename, mode) as f:
            for line in lines:
                f.write(line + "\r\n")

    def flush(self):
        # force l'ecriture du buffer courant sur la SD, meme s'il n'a pas
        # atteint flush_every -- appele automatiquement par log() a chaque
        # changement de phase de vol (voir plus bas)
        if self._pending:
            self._write_lines_now(self._pending)
            self._pending = []

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

    def _formatter_compact(self, timestamp, phase, accel, gyro, pression_pa,
                            temp_c, alt_baro_m, lat, lon, alt_gps_m,
                            z_kalman_m, vz_kalman_ms, batt_v):
        # memes 17 champs que _formatter, precision reduite : c'est la
        # version envoyee par LoRa (main.py), pas celle ecrite sur la SD.
        # Objectif : garder TOUTE l'info utile a une reconstruction du vol
        # (c'est la seule copie qui survit si la fusee ou la carte SD est
        # perdue) tout en reduisant le temps d'antenne, qui bloquait la
        # boucle principale (mesure : ~280 ms avec la precision complete,
        # str() sur un float Python peut sortir 15+ chiffres significatifs)
        ax, ay, az = accel
        gx, gy, gz = gyro
        return (
            f"{timestamp:.2f},{phase},{ax:.2f},{ay:.2f},{az:.2f},"
            f"{gx:.1f},{gy:.1f},{gz:.1f},{pression_pa:.1f},{temp_c:.1f},"
            f"{alt_baro_m:.1f},{lat:.5f},{lon:.5f},{alt_gps_m:.1f},"
            f"{z_kalman_m:.2f},{vz_kalman_ms:.2f},{batt_v:.2f}"
        )

    def log(self, timestamp, phase, accel, gyro, pression_pa, temp_c,
             alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms,
             batt_v):
        ligne = self._formatter(timestamp, phase, accel, gyro, pression_pa,
                                 temp_c, alt_baro_m, lat, lon, alt_gps_m,
                                 z_kalman_m, vz_kalman_ms, batt_v)
        self._pending.append(ligne)

        phase_changed = phase != self._last_phase
        self._last_phase = phase

        if phase_changed or len(self._pending) >= self.flush_every:
            self.flush()