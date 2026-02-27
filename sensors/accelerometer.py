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

class IMUTracker:

    def __init__(self):

        # Orientation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Velocity
        self.vx = 0.0
        self.vy = 0.0

        # Position
        self.x = 0.0
        self.y = 0.0

        # Biases
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)

        self.last_time = time.time()

        # Parameters
        self.alpha = 0.98
        self.max_speed = 1.5  # m/s
        self.stationary_threshold_accel = 0.08  # g deviation
        self.stationary_threshold_gyro = 1.0  # deg/s
        self.gravity = 9.81

    # -------------------------------
    # CALIBRATION
    # -------------------------------

    def calibrate(self, imu, samples=500):

        print("Calibrating IMU so it doesn't joycon drift... keep deboi still.")

        gyro_data = []
        accel_data = []

        for _ in range(samples):
            ax, ay, az = imu.get_accel()
            gx, gy, gz = imu.get_gyro()

            gyro_data.append([gx, gy, gz])
            accel_data.append([ax, ay, az])

            time.sleep(0.005)

        gyro_data = np.array(gyro_data)
        accel_data = np.array(accel_data)

        self.gyro_bias = np.mean(gyro_data, axis=0)

        # Assume stationary → accel measures gravity
        accel_mean = np.mean(accel_data, axis=0)
        self.accel_bias = accel_mean - np.array([0, 0, 1])

        print("Calibration complete.")
        print("Gyro bias:", self.gyro_bias)
        print("Accel bias:", self.accel_bias)

    # -------------------------------
    # UPDATE
    # -------------------------------

    def update(self, ax, ay, az, gx, gy, gz):

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Remove biases
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]

        ax -= self.accel_bias[0]
        ay -= self.accel_bias[1]
        az -= self.accel_bias[2]

        # Convert gyro to radians
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        # Accelerometer angles
        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        # Complementary filter
        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * accel_pitch

        # Yaw integration
        self.yaw += gz * dt

        # Convert accel to m/s²
        ax *= self.gravity
        ay *= self.gravity
        az *= self.gravity

        # Remove gravity component
        ax_world = ax * math.cos(self.pitch) + az * math.sin(self.pitch)
        ay_world = ay * math.cos(self.roll) - az * math.sin(self.roll)

        # -------------------------------
        # ZERO VELOCITY DETECTION
        # -------------------------------

        accel_mag = math.sqrt(ax_world**2 + ay_world**2)
        gyro_mag = math.degrees(abs(gz))

        is_stationary = (
            abs(accel_mag) < self.stationary_threshold_accel * self.gravity
            and gyro_mag < self.stationary_threshold_gyro
        )

        if is_stationary:
            self.vx = 0.0
            self.vy = 0.0
        else:
            self.vx += ax_world * dt
            self.vy += ay_world * dt

            # Velocity clamp
            speed = math.sqrt(self.vx**2 + self.vy**2)
            if speed > self.max_speed:
                scale = self.max_speed / speed
                self.vx *= scale
                self.vy *= scale

        # Integrate position
        self.x += self.vx * dt
        self.y += self.vy * dt

    # -------------------------------
    # OUTPUT
    # -------------------------------

    def get_pose(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": 0.0,
            "yaw": math.degrees(self.yaw),
            "pitch": math.degrees(self.pitch),
            "roll": math.degrees(self.roll)
        }
