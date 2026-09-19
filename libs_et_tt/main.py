# buzzer sur GP3 : allume en continu de PRE_LAUNCH a LANDED
import battery, datalog, kalman, sensors, state_machine, telemetry
import board
import busio
import digitalio
import time

DT = 0.01
TELEMETRY_PERIOD = 1.0
GROUND_PRESSURE_SAMPLES = 50   # mesures moyennees pour la ref baro sol
KALMAN_UPDATE_PERIOD = 0.22    # periode de correction barometrique

# recalage continu du biais accelerometrique pendant PRE_LAUNCH. Au repos, la
# force specifique lue sur l'axe longitudinal EST le biais, quelle que soit
# l'orientation. Ce recalage rend le vol insensible a l'assiette de la fusee
# au moment de la calibration initiale : teste calibre a 90 degres (fusee
# couchee), b converge quand meme a 9.807 en 20 s d'attente. Il est gele des
# qu'un echantillon depasse le seuil de decollage, pour qu'un choc ou le
# debut de la poussee ne vienne jamais polluer b.
ALPHA_BIAIS = 0.02             # tau = 1.4 s a 35 Hz

DEBUG_FREQ = True     # etat de vol sur le REPL, 1x/s ; False au jour J
BUZZER_ENABLED = True  # False au banc pour couper le son sans changer la logique


def compute_altitude(pressure_hpa, sea_level_hpa):
    # formule de adafruit_bmp3xx.altitude, appliquee a une pression deja lue
    # pour eviter une 2e conversion du BMP rien que pour l'altitude
    return 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** 0.1903)


sta = state_machine.StateMachine()
sta.state = "SETUP"

buzzer = digitalio.DigitalInOut(board.GP3)
buzzer.direction = digitalio.Direction.OUTPUT
buzzer.value = False


def set_buzzer(state):
    # seul point d'ecriture sur GP3 : BUZZER_ENABLED coupe le son partout
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
        ground_pressure_samples.append(sen.baro_pt[0])
        time.sleep(0.05)
    sen.bmp.sea_level_pressure = (sum(ground_pressure_samples) / GROUND_PRESSURE_SAMPLES)

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

# valeurs de repli : si la toute premiere lecture d'un capteur echoue, la
# boucle doit quand meme pouvoir tourner
baro_hpa, baro_temp = sen.bmp.sea_level_pressure, 15.0
baro_alt = 0.0
imu_accel = (0.0, 0.0, 9.81)
gyro = (0.0, 0.0, 0.0)
batv = 0.0
# derniere position GPS connue ; ne retombe pas a 0.0/0 si le fix est perdu
lat, lon, alt, sat = 0.0, 0.0, 0.0, 0

n_err_baro = 0
n_err_imu = 0
n_err_log = 0

while True:
    # Chaque sous-systeme est protege SEPAREMENT, et l'estimation comme la
    # machine d'etat ne sont JAMAIS sautees. Avec un try/except unique autour
    # de toute la boucle, un bus I2C bloque (esclave qui tient SDA bas sous
    # vibration) figeait definitivement la machine d'etat : plus de
    # transition, plus de detection d'apogee, plus de log.
    sec = time.monotonic()

    baro_ok = True
    try:
        baro_hpa, baro_temp = sen.baro_pt
        baro_alt = compute_altitude(baro_hpa, sen.bmp.sea_level_pressure)
    except Exception:
        baro_ok = False
        n_err_baro += 1

    try:
        imu_accel = sen.imu_accel
        gyro = sen.imu_gyro
    except Exception:
        n_err_imu += 1   # on conserve la derniere mesure valide

    if sta.state == "DESCENT" or sta.state == "LANDED" or sta.state == "PRE_LAUNCH":
        try:
            gps = sen.gps_data
            if gps is not None:
                lat, lon, alt, sat = gps
        except Exception:
            pass

    try:
        batv = bat.tension
    except Exception:
        pass

    # ---- estimation : jamais sautee -------------------------------------
    dt_pred = sec - t_prev_pred
    t_prev_pred = sec
    kal_h, kal_v, kal_b = kal.prediction(dt_pred, imu_accel[2])

    if baro_ok and sec - t_prev_upd >= KALMAN_UPDATE_PERIOD:
        kal_h, kal_v, kal_b = kal.update(baro_alt)
        t_prev_upd = sec

    # au sol et sous le seuil de decollage : h et v sont connus exactement,
    # on les force plutot que de laisser la moindre derive s'accumuler
    # pendant l'attente au pas de tir. Des le premier echantillon au-dessus
    # du seuil on relache, pour ne pas perdre les 86 ms d'integration que
    # dure la confirmation du decollage.
    if sta.state == "PRE_LAUNCH" and imu_accel[2] < sta.accelLimitForBoost:
        kal.b += ALPHA_BIAIS * (imu_accel[2] - kal.b)
        kal.h = 0.0
        kal.v = 0.0
        kal_h, kal_v, kal_b = 0.0, 0.0, kal.b

    # ---- machine d'etat : jamais sautee ---------------------------------
    sta.update(sec, imu_accel[2], kal_v, kal_h, baro_alt)

    # ---- journalisation --------------------------------------------------
    try:
        dat.log(
            sec, sta.state, imu_accel, gyro,
            baro_hpa, baro_temp, baro_alt,
            lat, lon, alt,
            kal_h, kal_v, batv
        )
    except Exception:
        n_err_log += 1

    # ---- radio : coupee entre le decollage et l'apogee --------------------
    # chaque emission bloque la boucle ~180 ms. En laissant la radio active
    # pendant la montee, la RMSE d'altitude en COAST passait de 0.31 m a
    # 3.55 m : prediction() se retrouve a integrer 0.5*a*dt^2 sur un pas de
    # 0.21 s en pleine phase de jerk eleve. Accessoirement, 175 ms par
    # seconde font 17.5 % de cycle de service sur une bande 868.0 MHz
    # limitee a 1 %.
    if (sta.state in ("PRE_LAUNCH", "DESCENT", "LANDED")
            and sec - t_prev_tel >= TELEMETRY_PERIOD):
        try:
            to_send = dat._formatter_compact(
                sec, sta.state, imu_accel, gyro,
                baro_hpa, baro_temp, baro_alt,
                lat, lon, alt,
                kal_h, kal_v, batv
            )
            tel.send(to_send)
        except Exception:
            pass
        t_prev_tel = sec

    if DEBUG_FREQ and sec - t_prev_debug >= 1.0:
        # go/no-go avant allumage : en PRE_LAUNCH, h_kalman doit rester a
        # +/- 1 m de 0 et v a +/- 0.2 m/s, et biais doit etre proche de 9.81
        print(
            "etat=", sta.state,
            " h_kal=", round(kal_h, 2), "m",
            " h_baro=", round(baro_alt, 2), "m",
            " v=", round(kal_v, 2), "m/s",
            " biais=", round(kal.b, 3),
            " p_hh=", round(kal.p_hh, 4),
            " rejets=", kal.n_rejets,
            " err(baro/imu/log)=", n_err_baro, n_err_imu, n_err_log,
            " batt=", batv, "V",
        )
        t_prev_debug = sec