import serial
import struct
import math
import numpy as np
import json

# SERIAL SETUP
# ---------------------------
ser = serial.Serial("/dev/ttyUSB0", 250000, timeout=1)

START = b"START"
STRUCT_FORMAT = "<fffhfff"   # lat, lon, yaw_raw, sat, mx, my, mz
SIZE = struct.calcsize(STRUCT_FORMAT)


# ---------------------------
# MAIN LOOP
# ---------------------------
while True:

    # Wait for START header
    if ser.read(5) != START:
        continue

    # Read struct data
    data = ser.read(SIZE)
    if len(data) != SIZE:
        continue

    lat, lon, yaw_raw, sat, mx_raw, my_raw, mz_raw = struct.unpack(STRUCT_FORMAT, data)

    print(f"\nLat={lat:.6f}, Lon={lon:.6f}, Sat={sat}")
    print(f"Raw Yaw (Arduino)     = {yaw_raw:.2f}°")


