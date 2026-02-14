'''
NOTE: This is meant to run on the following hardware:
-Raspberry Pi 2W zero
-Raspberry Pi Camera Module 3 (connected using HBV-Raspberry-150FPC
'''

import subprocess
import time
from datetime import datetime
from pathlib import Path

IMAGE_DIR = Path("images")
IMAGE_DIR.mkdir(exist_ok=True)

def capture_image():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = IMAGE_DIR / f"frame_{timestamp}.jpg"
    #250x250 so it's suitable for model
    command = [
        "libcamera-still",
        "-o", str(filename),
        "--width", "250",
        "--height", "250",
        "--nopreview",
        "--timeout", "500"
    ]

    subprocess.run(command, check=True)

    return filename
