#buzzer sur gpio
import battery, datalog, kalman, sensors, state_machine, telemetry
import board
import busio
import time

sta = state_machine.StateMachine()
sta.state = "SETUP"

DT = 0.01
TELEMETRY_PERIOD = 0.22  # meme cadence que la maj barometrique du Kalman

SPI = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
radio_cs_pin, radio_reset_pin = board.GP8, board.GP9

bat = battery.Battery()
dat = datalog.Datalog(SPI)
kal = kalman.Kalman(DT)
sen = sensors.Sensors()

kal_cal = kal.calibrate(300)
tel = telemetry.Telemetry(SPI, radio_cs_pin, radio_reset_pin)

time.sleep(5)
sta.state = "PRE_LAUNCH"

# on stocke des TIMESTAMPS ici
t_prev_pred = time.monotonic()
t_prev_upd = time.monotonic()
t_prev_tel = time.monotonic()

while True:
    baro_pa = sen.baro_pa
    baro_alt = sen.baro_alt
    baro_temp = sen.baro_temp
    imu_accel = sen.imu_accel
    gyro = sen.imu_gyro

    gps = sen.gps_data
    if gps is None:
        lat, lon, alt, sat = 0.0, 0.0, 0.0, 0
    else:
        lat, lon, alt, sat = gps

    batv = bat.tension
    sec = time.monotonic()

    # --- prediction Kalman : dt recalcule a chaque tour, timestamp reinjecte ---
    dt_pred = sec - t_prev_pred
    t_prev_pred = sec
    kal_h, kal_v, kal_b = kal.prediction(dt_pred, imu_accel[2])

    # --- recalage barometrique a ~0,22 s (limite par la freq du BMP388) ---
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