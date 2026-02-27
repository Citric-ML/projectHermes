import smbus2
import time
import math
import numpy as np

MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT = 0x3B
GYRO_XOUT = 0x43


class MPU6050:

    def __init__(self, bus_id=1):
        self.bus = smbus2.SMBus(bus_id)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

    def read_word(self, reg):
        high = self.bus.read_byte_data(MPU_ADDR, reg)
        low = self.bus.read_byte_data(MPU_ADDR, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def get_accel(self):
        ax = self.read_word(ACCEL_XOUT) / 16384.0
        ay = self.read_word(ACCEL_XOUT + 2) / 16384.0
        az = self.read_word(ACCEL_XOUT + 4) / 16384.0
        return ax, ay, az

    def get_gyro(self):
        gx = self.read_word(GYRO_XOUT) / 131.0
        gy = self.read_word(GYRO_XOUT + 2) / 131.0
        gz = self.read_word(GYRO_XOUT + 4) / 131.0
        return gx, gy, gz
