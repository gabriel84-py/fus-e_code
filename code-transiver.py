import board
import busio
import digitalio
import adafruit_rfm9x
import adafruit_bmp3xx
import time
import adafruit_gps
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import sdcardio
import storage
import adafruit_sdcard
i2c = busio.I2C(board.GP15, board.GP14)
sensor = LSM6DSOX(i2c)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
cs_port_rfm = board.GP8
cs_rfm = digitalio.DigitalInOut(cs_port_rfm)
reset_rfm = digitalio.DigitalInOut(board.GP9)
rfm9x = adafruit_rfm9x.RFM9x(spi, cs_rfm, reset_rfm, 868.0, baudrate=1000000)

cs_sd = digitalio.DigitalInOut(board.GP1)
sdcard = adafruit_sdcard.SDCard(spi, cs_sd)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")
with open("/sd/test.txt", "w") as f:
    f.write("Hello world!\r\n")
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
RX = board.GP13
TX = board.GP12
uart = busio.UART(TX, RX, baudrate=9600, timeout=30)
gps = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,1000')
last_print = time.monotonic()
while True:
    rfm9x.send(str(bmp.altitude))
    print("Alt: {:6.1f}".format(bmp.altitude))
    gps.update()
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        if not gps.has_fix:
            print('Waiting for fix...')
            continue
        print('=' * 40)  # Print a separator line.
        print('Latitude: {0:.6f} degrees'.format(gps.latitude))
        print('Longitude: {0:.6f} degrees'.format(gps.longitude))
    accel_x, accel_y, accel_z = sensor.acceleration
    print(f"Acceleration: X:{accel_x:.2f}, Y: {accel_y:.2f}, Z: {accel_z:.2f} m/s^2")
    gyro_x, gyro_y, gyro_z = sensor.gyro
    print(f"Gyro X:{gyro_x:.2f}, Y: {gyro_y:.2f}, Z: {gyro_z:.2f} radians/s")
    print("")
    time.sleep(1)
