'''
NOTE: This is meant to run on the following hardware:
-Raspberry Pi 2W
-U.S. 100 ultrasonic sensor
-level shifter
'''
import RPi.GPIO as GPIO
import time

TRIG = 24
ECHO = 23

def setup_sonar():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.output(TRIG, False)
    time.sleep(0.5)  # allow sensor to settle


def get_distance():
    # Send 10µs pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for echo start
    timeout = time.time() + 0.05  # 50ms timeout
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None  # timeout error

    # Wait for echo end
    timeout = time.time() + 0.05
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None  # timeout error

    pulse_duration = pulse_end - pulse_start

    # Speed of sound ≈ 34300 cm/s
    distance_cm = (pulse_duration * 34300) / 2

    return distance_cm
