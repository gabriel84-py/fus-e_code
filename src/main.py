#buzzer sur gpio
import apogee, battery, datalog, kalman, sensors, state_machine, telemetry
import board
import busio
import digitalio
import adafruit_rfm9x
import storage
import adafruit_sdcard
import time

sta = state_machine.StateMachine()
sta.state = "SETUP"

DT = 0.01
SPI = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
radio_cs_pin, radio_reset_pin = 8, 9

apo = apogee.ApogeeDetection()
bat = battery.Battery()
dat = datalog.Datalog(SPI)
kal = kalman.Kalman(DT)
kal_cal = kal.calibrate(300)
sen = sensors.Sensors()
tel = telemetry.Telemetry(SPI, radio_cs_pin, radio_reset_pin)

time.sleep(5)
sta.state = "PRE_LAUNCH"
dt_pred = time.monotonic()
dt_upd = time.monotonic()

while True:
    baro_pa = sen.baro_pa
    baro_alt = sen.baro_alt
    baro_temp = sen.baro_temp
    imu_accel = sen.imu_accel
    gyro = sen.imu_gyro
    lat, lon, alt, sat = sen.gps_data
    batv = bat.tension
    sec = time.monotonic()

    dt_pred = sec - dt_pred
    kal_h, kal_v, kal_b = kal.prediction(dt_pred, imu_accel[2])
    if sec - dt_upd >= 0.22:
        kal_h, kal_v, kal_b = kal.update(baro_alt)
        dt_upd = sec

    sta.update(sec, imu_accel[2], kal_v, kal_h)

    dat.log(
        sec, sta.state, imu_accel, gyro,
        baro_pa, baro_temp, baro_alt,
        lat, lon, alt,
        kal_h, kal_v, batv
    )

    to_send = dat._formatter(
        sec, sta.state, imu_accel, gyro,
        baro_pa, baro_temp, baro_alt,
        lat, lon, alt,
        kal_h, kal_v, batv
    )

    if sec - dt_upd >= 0.22:
        tel.send(to_send)


