# buzzer sur GP3 : allume en continu de PRE_LAUNCH a LANDED
import battery, datalog, kalman, sensors, state_machine, telemetry
import board
import busio
import digitalio
import time

DT = 0.01
# marge large au-dessus de l'airtime LoRa mesure sur banc (~180 ms avec le
# format compact) : evite que l'envoi radio se redeclenche a chaque tour
# de boucle si un envoi depasse la periode prevue
TELEMETRY_PERIOD = 1.0
GROUND_PRESSURE_SAMPLES = 50  # mesures moyennees pour la ref baro sol

DEBUG_FREQ = True   # affiche un etat de vol basique sur le REPL, 1x/s ; False au jour J pour zero overhead
BUZZER_ENABLED = False  # False au banc pour couper le son sans changer la logique de vol


def compute_altitude(pressure_hpa, sea_level_hpa):
    # formule de adafruit_bmp3xx.altitude, appliquee a une pression deja lue
    # (baro_pa) pour eviter une 2e lecture I2C de la pression rien que pour
    # l'altitude (cf sensors.baro_alt)
    return 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** 0.1903)


sta = state_machine.StateMachine()
sta.state = "SETUP"

buzzer = digitalio.DigitalInOut(board.GP3)
buzzer.direction = digitalio.Direction.OUTPUT
buzzer.value = False


def set_buzzer(state):
    # seul point d'ecriture sur GP3 : BUZZER_ENABLED coupe le son partout
    # (bip de vol comme alarme d'echec init) sans dupliquer la condition
    buzzer.value = state and BUZZER_ENABLED


SPI = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
radio_cs_pin, radio_reset_pin = board.GP8, board.GP9

# init complete + calage baro, protegee : une panne ici bloque au sol avec
# une alarme sonore plutot que de decoller avec une chaine de mesure
# incomplete
try:
    sen = sensors.Sensors()
    bat = battery.Battery()
    dat = datalog.Datalog(SPI)
    tel = telemetry.Telemetry(SPI, radio_cs_pin, radio_reset_pin)

    kal = kalman.Kalman(DT, sen)
    kal_cal = kal.calibrate(300)

    # pression sol = reference 0 m, fusee posee au sol, avant le sleep(5)
    ground_pressure_samples = []
    for _ in range(GROUND_PRESSURE_SAMPLES):
        ground_pressure_samples.append(sen.baro_pa)
        time.sleep(0.05)
    sen.bmp.sea_level_pressure = sum(ground_pressure_samples) / GROUND_PRESSURE_SAMPLES

except Exception as exc:
    # bips rapides en boucle : motif distinct de tout etat de vol normal
    print("ECHEC INIT:", exc)
    beep_state = False
    while True:
        beep_state = not beep_state
        set_buzzer(beep_state)
        time.sleep(0.1)

time.sleep(5)
sta.state = "PRE_LAUNCH"

# bip continu de PRE_LAUNCH a LANDED, un seul passage a True suffit
set_buzzer(True)

t_prev_pred = time.monotonic()
t_prev_upd = time.monotonic()
t_prev_tel = time.monotonic()
t_prev_debug = time.monotonic()

# derniere position GPS connue ; ne retombe pas a 0.0/0 si le fix est perdu
lat, lon, alt, sat = 0.0, 0.0, 0.0, 0

while True:
    # boucle entierement protegee : un incident transitoire sur un
    # sous-systeme (I2C, radio, SD) ne doit jamais arreter la machine
    # d'etat ni le buzzer
    try:
        baro_pa = sen.baro_pa
        baro_alt = compute_altitude(baro_pa, sen.bmp.sea_level_pressure)
        baro_temp = sen.baro_temp
        imu_accel = sen.imu_accel
        gyro = sen.imu_gyro

        if sta.state == "DESCENT" or sta.state == "LANDED" or sta.state == "PRE_LAUNCH":
            gps = sen.gps_data
            if gps is not None:
                lat, lon, alt, sat = gps

        batv = bat.tension
        sec = time.monotonic()

        dt_pred = sec - t_prev_pred
        t_prev_pred = sec
        kal_h, kal_v, kal_b = kal.prediction(dt_pred, imu_accel[2])

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
            # version compacte pour la radio (memes champs, moins de
            # decimales) : seule copie qui survit si la fusee/la SD est
            # perdue, donc on garde tous les champs
            to_send = dat._formatter_compact(
                sec, sta.state, imu_accel, gyro,
                baro_pa, baro_temp, baro_alt,
                lat, lon, alt,
                kal_h, kal_v, batv
            )
            tel.send(to_send)
            t_prev_tel = sec

        if DEBUG_FREQ and sec - t_prev_debug >= 1.0:
            print("etat=", sta.state, " h=", round(kal_h, 2), "m v=", round(kal_v, 2), "m/s batt=", batv, "V")
            t_prev_debug = sec

    except Exception as exc:
        print("ERREUR BOUCLE:", exc)
        continue