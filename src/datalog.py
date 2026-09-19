import board
import digitalio
import os
import time
import storage
import adafruit_sdcard

# timestamp_s, phase, a_lat1, a_lat2, a_long, gx, gy, gz, pression_hpa,
# temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v


class Datalog:
    def __init__(self, spi, flush_every=10):
        # flush_every : nb de lignes bufferisees en memoire avant
        # reouverture/fermeture reelle du fichier sur la SD. Chaque
        # ouverture/fermeture coute cher sur SD SPI (~30 ms mesures sur
        # banc) ; grouper N lignes divise ce cout par N. En cas de coupure
        # entre deux flush on perd au plus (flush_every - 1) lignes, jamais
        # tout le vol. flush() est aussi force a chaque changement de phase.
        cs_sd = digitalio.DigitalInOut(board.GP1)
        self.sdcard = adafruit_sdcard.SDCard(spi, cs_sd)
        vfs = storage.VfsFat(self.sdcard)
        try:
            storage.mount(vfs, "/sd")
        except (RuntimeError, OSError):
            # deja monte : arrive apres un soft reset (Ctrl-D au REPL). Sans
            # ce garde-fou l'init partait dans l'alarme sonore alors que le
            # materiel va parfaitement bien.
            pass

        self.filename = self._next_filename()
        self.flush_every = flush_every
        self._pending = []
        self._last_phase = None

        self._write_lines_now(
            ["timestamp_s, phase, a_lat1, a_lat2, a_long, gx, gy, gz, "
             "pression_hpa, temp_c, alt_baro_m, lat, lon, alt_gps_m, "
             "z_kalman_m, vz_kalman_ms, batt_v"],
            mode="w",  # fichier neuf, en-tete seule
        )

    @staticmethod
    def _next_filename():
        """Index libre, scanne sur la carte.

        L'ancien nom data_{time.time()}.csv etait dangereux : le RP2350 n'a
        pas de RTC sauvegardee, time.time() repart de la meme epoque a chaque
        boot et la sequence d'init dure sensiblement la meme duree, donc deux
        demarrages produisaient tres probablement le MEME nom, reouvert en
        mode "w" et donc ecrase. Un simple reboot apres l'atterrissage
        (faux contact batterie) effacait le vol.
        """
        idx = 0
        try:
            for f in os.listdir("/sd"):
                if f.startswith("data_") and f.endswith(".csv"):
                    try:
                        idx = max(idx, int(f[5:-4]) + 1)
                    except ValueError:
                        pass
        except OSError:
            pass
        return "/sd/data_%03d.csv" % idx

    def _write_lines_now(self, lines, mode="a"):
        with open(self.filename, mode) as f:
            for line in lines:
                f.write(line + "\r\n")

    def flush(self):
        if self._pending:
            self._write_lines_now(self._pending)
            self._pending = []

    def _formatter(self, timestamp, phase, accel, gyro, pression_hpa,
                   temp_c, alt_baro_m, lat, lon, alt_gps_m,
                   z_kalman_m, vz_kalman_ms, batt_v):
        a1, a2, a3 = accel
        gx, gy, gz = gyro
        return (
            f"{timestamp}, {phase}, {a1}, {a2}, {a3}, {gx}, {gy}, {gz}, "
            f"{pression_hpa}, {temp_c}, {alt_baro_m}, {lat}, {lon}, "
            f"{alt_gps_m}, {z_kalman_m}, {vz_kalman_ms}, {batt_v}"
        )

    def _formatter_compact(self, timestamp, phase, accel, gyro, pression_hpa,
                           temp_c, alt_baro_m, lat, lon, alt_gps_m,
                           z_kalman_m, vz_kalman_ms, batt_v):
        # memes 17 champs, precision reduite : version envoyee par LoRa.
        # Objectif : garder TOUTE l'info utile a une reconstruction du vol
        # (seule copie qui survit si la fusee ou la carte SD est perdue) tout
        # en reduisant le temps d'antenne. 103 octets pour 251 autorises.
        a1, a2, a3 = accel
        gx, gy, gz = gyro
        return (
            f"{timestamp:.2f},{phase},{a1:.2f},{a2:.2f},{a3:.2f},"
            f"{gx:.1f},{gy:.1f},{gz:.1f},{pression_hpa:.2f},{temp_c:.1f},"
            f"{alt_baro_m:.1f},{lat:.5f},{lon:.5f},{alt_gps_m:.1f},"
            f"{z_kalman_m:.2f},{vz_kalman_ms:.2f},{batt_v:.2f}"
        )

    def log(self, timestamp, phase, accel, gyro, pression_hpa, temp_c,
            alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms,
            batt_v):
        ligne = self._formatter(timestamp, phase, accel, gyro, pression_hpa,
                                temp_c, alt_baro_m, lat, lon, alt_gps_m,
                                z_kalman_m, vz_kalman_ms, batt_v)
        self._pending.append(ligne)

        phase_changed = phase != self._last_phase
        self._last_phase = phase

        if phase_changed or len(self._pending) >= self.flush_every:
            self.flush()