from ..sensors.accelerometer import MPU6050
import time

accel = MPU6050()

for i in range(100):
    print(accel.get_accel())
    print(accel.get_gyro())
    time.sleep(0.5)
