# buzzer sur GP3 : allume en continu de PRE_LAUNCH a LANDED (voir plus bas)
import battery, datalog, kalman, sensors, state_machine, telemetry
import board
import busio
import digitalio
import time

DT = 0.01
TELEMETRY_PERIOD = 0.22  # meme cadence que la maj barometrique du Kalman
GROUND_PRESSURE_SAMPLES = 20  # nb de mesures moyennees pour la ref baro sol
DEBUG_FREQ = True  # affiche la freq de boucle reelle sur le REPL, 1x/s ;
                    # a couper (False) au jour J

sta = state_machine.StateMachine()
sta.state = "SETUP"

buzzer = digitalio.DigitalInOut(board.GP3)
buzzer.direction = digitalio.Direction.OUTPUT
buzzer.value = False  # reste eteint pendant l'init/calibration au sol

SPI = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
radio_cs_pin, radio_reset_pin = board.GP8, board.GP9

# --- initialisation capteurs/peripheriques + calage baro, avec diagnostic ---
# tout ce qui peut echouer au pas de tir (capteur mal connecte, SD absente,
# radio qui ne repond pas) est regroupe ici : si ca casse, on le signale au
# buzzer et on bloque plutot que de decoller avec une chaine de mesure
# incomplete. C'est volontairement un except large : a ce stade on ne fait
# aucune distinction fine entre les pannes, on veut juste NE PAS voler.
try:
    sen = sensors.Sensors()
    bat = battery.Battery()
    dat = datalog.Datalog(SPI)
    tel = telemetry.Telemetry(SPI, radio_cs_pin, radio_reset_pin)

    kal = kalman.Kalman(DT, sen)
    kal_cal = kal.calibrate(300)

    # calage barometrique : pression sol mesuree ICI = reference 0 m.
    # fusee posee au sol, avant le sleep(5) et la mise en config de vol.
    ground_pressure_samples = []
    for _ in range(GROUND_PRESSURE_SAMPLES):
        ground_pressure_samples.append(sen.baro_pa)
        time.sleep(0.05)  # laisse le temps au BMP388 de rafraichir sa mesure
    sen.bmp.sea_level_pressure = sum(ground_pressure_samples) / GROUND_PRESSURE_SAMPLES

except Exception as exc:
    # motif distinct de tout ce qui arrive en vol : bips rapides en boucle
    # infinie, pour qu'une panne d'init soit immediatement reconnaissable
    # au pas de tir et ne soit jamais confondue avec un etat de vol normal
    print("ECHEC INIT:", exc)
    while True:
        buzzer.value = not buzzer.value
        time.sleep(0.1)

time.sleep(5)
sta.state = "PRE_LAUNCH"

# buzzer allume en continu a partir d'ici, sur toutes les phases de vol
# (PRE_LAUNCH -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED)
buzzer.value = True

# on stocke des TIMESTAMPS ici
t_prev_pred = time.monotonic()
t_prev_upd = time.monotonic()
t_prev_tel = time.monotonic()
t_prev_freq = time.monotonic()
loop_count = 0

# derniere position GPS connue ; reste a 0.0/0 tant qu'aucun fix n'a ete pris
lat, lon, alt, sat = 0.0, 0.0, 0.0, 0

while True:
    baro_pa = sen.baro_pa
    baro_alt = sen.baro_alt
    baro_temp = sen.baro_temp
    imu_accel = sen.imu_accel
    gyro = sen.imu_gyro

    if sta.state == "DESCENT" or sta.state == "LANDED" or sta.state == "PRE_LAUNCH":
        gps = sen.gps_data
        # si gps is None (fix perdu), on garde la derniere
        # position connue au lieu de retomber a 0.0/0
        if gps is not None:
            lat, lon, alt, sat = gps

    batv = bat.tension
    sec = time.monotonic()

    # --- prediction Kalman : dt recalcule a chaque tour, timestamp reinjecte ---
    dt_pred = sec - t_prev_pred
    t_prev_pred = sec
    kal_h, kal_v, kal_b = kal.prediction(dt_pred, imu_accel[2])

    # --- recalage barometrique a 0,22 s (limite par la freq du BMP388) ---
    if sec - t_prev_upd >= 0.22:
        kal_h, kal_v, kal_b = kal.update(baro_alt)
        t_prev_upd = sec

    sta.update(sec, imu_accel[2], kal_v, kal_h)

    dat.log(
        sec, sta.state, imu_accel, gyro,
        baro_pa, baro_temp, baro_alt,
        lat, lon, alt,
        kal_h, kal_v, batv
    )

    if sec - t_prev_tel >= TELEMETRY_PERIOD:
        to_send = dat._formatter(
            sec, sta.state, imu_accel, gyro,
            baro_pa, baro_temp, baro_alt,
            lat, lon, alt,
            kal_h, kal_v, batv
        )
        tel.send(to_send)
        t_prev_tel = sec

    # --- mesure de la freq de boucle reelle, pour le banc ---
    # print() sur le REPL uniquement, jamais ecrit sur la SD : ca ne doit
    # pas ajouter d'I/O disque et fausser la mesure qu'on cherche a faire
    if DEBUG_FREQ:
        loop_count += 1
        if sec - t_prev_freq >= 1.0:
            print("freq boucle:", loop_count, "Hz (etat=", sta.state, ")")
            loop_count = 0
            t_prev_freq = sec