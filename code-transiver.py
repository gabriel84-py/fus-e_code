import board
import busio
import digitalio
import adafruit_rfm9x
import adafruit_bmp3xx
import time
import adafruit_gps
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32
import sdcardio
import storage
import adafruit_sdcard

# Setup I2C
i2c = busio.I2C(board.GP15, board.GP14)

def setup_imu():
    sensor = LSM6DSO32(i2c)
    return sensor

sensor = setup_imu()

# Setup SPI
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

def setup_rfm9x():
    cs_port_rfm = board.GP8
    cs_rfm = digitalio.DigitalInOut(cs_port_rfm)
    reset_rfm = digitalio.DigitalInOut(board.GP9)
    rfm9x = adafruit_rfm9x.RFM9x(spi, cs_rfm, reset_rfm, 868.0, baudrate=1000000)
    return rfm9x

rfm9x = setup_rfm9x()

def setup_sdcard():
    cs_sd = digitalio.DigitalInOut(board.GP1)
    sdcard = adafruit_sdcard.SDCard(spi, cs_sd)
    vfs = storage.VfsFat(sdcard)
    storage.mount(vfs, "/sd")
    return sdcard

sdcard = setup_sdcard()

def write_to_sdcard(something, filename):
    with open(filename, "w") as f:
        f.write(something + "\r\n")

def setup_bmp():
    bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
    return bmp

bmp = setup_bmp()

def setup_gps():
    RX = board.GP13
    TX = board.GP12
    uart = busio.UART(TX, RX, baudrate=9600, timeout=30)
    gps = adafruit_gps.GPS(uart, debug=False)
    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    gps.send_command(b'PMTK220,1000')
    last_print = time.monotonic()
    return gps, last_print

gps, last_print = setup_gps()

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
        sats = gps.satellites if gps.satellites is not None else 0
        print('Satellites: {0}'.format(sats))
    accel_x, accel_y, accel_z = sensor.acceleration
    print(f"Acceleration: X:{accel_x:.2f}, Y: {accel_y:.2f}, Z: {accel_z:.2f} m/s^2")
    gyro_x, gyro_y, gyro_z = sensor.gyro
    print(f"Gyro X:{gyro_x:.2f}, Y: {gyro_y:.2f}, Z: {gyro_z:.2f} radians/s")
    print("")
    time.sleep(1)