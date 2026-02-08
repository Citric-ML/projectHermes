'''
sends direction vector to arduino to execute via logic switcher
input:
    Astar path coords on converged grid
output:
    list (for one movement):
        angle of direction for drone
        time moving in one direction
        Speed of motors
'''
from converged import buildpathcoords
path = buildpathcoords()
#seperates flight path into chunks (based on the different angles it goes in)

import math

def separate_chunks(path):
    if len(path) < 2:
        return []

    chunks = []
    current_chunk = [path[0]]

    prev_dx = path[1][0] - path[0][0]
    prev_dy = path[1][1] - path[0][1]

    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]

        if (dx, dy) == (prev_dx, prev_dy):
            current_chunk.append(path[i])
        else:
            chunks.append((current_chunk, prev_dx, prev_dy))
            current_chunk = [path[i-1], path[i]]
            prev_dx, prev_dy = dx, dy

    chunks.append((current_chunk, prev_dx, prev_dy))
    return chunks



def find_angle(dx, dy):
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    return angle_deg
def find_speed():
    return 0.5  # meters per second (example)

def find_duration(chunk_length, speed, grid_resolution=0.2):
    distance = chunk_length * grid_resolution
    return distance / speed

def build_command_list(path):
    command_list = []
    chunks = separate_chunks(path)

    for chunk, dx, dy in chunks:
        angle = find_angle(dx, dy)
        speed = find_speed()
        duration = find_duration(len(chunk), speed)

        command_list.append([
            round(angle, 2),
            round(speed, 2),
            round(duration, 2)
        ])

    return command_list

#--------------------------
#Communicates with Aruduino
#--------------------------
'''
Requirements:
    Modules:
        needs pyserial (pip install pyserial)
    Pi:
        diable login shell over serial
        enable serial hardware
'''
import serial
import time

SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200

def send_command_list(command_list):
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        time.sleep(2)  # allow Arduino reset

        ser.write(b"CMD_START\n")

        for angle, speed, duration in command_list:
            line = f"{angle},{speed},{duration}\n"
            ser.write(line.encode("utf-8"))

        ser.write(b"CMD_END\n")
        ser.flush()
