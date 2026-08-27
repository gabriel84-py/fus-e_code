from src.sensors import Sensors

sensors = Sensors()
while True:
    print(f"Pression : {sensors.baro_pa}")
    print(f"Altitude : {sensors.baro_alt}")
    print(f"Acceleration : X:{sensors.imu_accel[0]}, Y: {sensors.imu_accel[1]}, Z : {sensors.imu_accel[2]},")
    print(f"Gyroscope : X:{sensors.imu_gyro[0]}, Y: {sensors.imu_gyro[1]}, Z : {sensors.imu_gyro[2]},")
    print(f"Position : Latitude :{sensors.gps_latlong[0]}, Longitude : {sensors.gps_latlong[1]}")
    print(f"Altitude GPS : {sensors.gps_alt}")
    print(f"Satellite : {sensors.gps_sat}")
    print('=' * 40 + '\n')
