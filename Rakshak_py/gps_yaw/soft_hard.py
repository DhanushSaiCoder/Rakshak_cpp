import serial
import struct
import math
import numpy as np
import json
import os

# ---------------------------
# SERIAL SETUP
# ---------------------------
ser = serial.Serial("/dev/ttyUSB0", 250000, timeout=1)

START = b"START"
STRUCT_FORMAT = "<fffhfff"   # lat, lon, yaw_raw, sat, mx, my, mz
SIZE = struct.calcsize(STRUCT_FORMAT)

# ==================================================================
#  FUNCTION: LOAD CALIBRATION + APPLY OR FALLBACK
# ==================================================================
calib_loaded = False
A = None
bx = by = bz = 0

def try_load_calibration():
    """Load calibration JSON only once."""
    global calib_loaded, A, bx, by, bz

    if calib_loaded:
        return calib_loaded

    CALIB_FILE = "mag_500_calibration.json"

    if not os.path.exists(CALIB_FILE):
        print("\n!! Calibration file NOT found → Using RAW yaw only\n")
        calib_loaded = False
        return False

    try:
        with open(CALIB_FILE, "r") as f:
            calib = json.load(f)

        # Hard-iron bias
        bx, by, bz = calib["hard_iron_bias"]

        # Soft-iron 3×3 matrix
        A = np.array(calib["soft_iron_matrix"])

        print("\nLoaded calibration:")
        print("Hard-iron bias:", bx, by, bz)
        print("Soft-iron matrix:\n", A)
        print("\nStarting calibrated yaw reading...\n")

        calib_loaded = True
        return True

    except Exception as e:
        print("\n!! Error loading calibration file:", e)
        print("!! Using RAW yaw only\n")
        calib_loaded = False
        return False


# ==================================================================
#  APPLY CALIBRATION + COMPUTE YAW
# ==================================================================
def get_yaw(mx_raw, my_raw, mz_raw):
    """Return calibrated yaw if calibration available, else raw yaw."""
    if not calib_loaded:
        return None   # signal to print raw yaw

    # Hard-iron correction
    t = np.array([
        mx_raw - bx,
        my_raw - by,
        mz_raw - bz
    ])

    # Soft-iron correction
    calibrated = A @ t
    mx_c, my_c, mz_c = calibrated

    # Compute calibrated yaw
    yaw_cal = math.degrees(math.atan2(-mx_c, my_c))
    if yaw_cal < 0:
        yaw_cal += 360

    return yaw_cal


# ==================================================================
#  MAIN LOOP
# ==================================================================
# Try loading calibration at program start
#try_load_calibration()

while True:

    # Wait for START header
    if ser.read(5) != START:
        continue

    # Read struct data
    data = ser.read(SIZE)
    if len(data) != SIZE:
        continue

    lat, lon, yaw_raw, sat, mx_raw, my_raw, mz_raw = struct.unpack(STRUCT_FORMAT, data)

    # Compute calibrated yaw OR None (fallback)
    yaw_cal = get_yaw(mx_raw, my_raw, mz_raw)

    print(f"\nLat={lat:.6f}, Lon={lon:.6f}, Sat={sat}")
    print(f"Raw Yaw (Arduino) = {yaw_raw:.2f}°")
   # print(f"Calibrated Yaw (Python) = {yaw_cal:.2f}°")

