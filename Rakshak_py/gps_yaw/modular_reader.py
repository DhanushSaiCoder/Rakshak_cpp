import serial
import struct
import math
import numpy as np
import json
import os
import time

class SensorReaderGPS:
    START = b"START"
    STRUCT_FORMAT = "<fffhfff"      # lat, lon, yaw_raw, sat, mx, my, mz
    SIZE = struct.calcsize(STRUCT_FORMAT)

    def __init__(self, port="/dev/ttyUSB1", baud=250000, timeout=1,
                 calib_file="mag_500_calibration.json"):

        self.ser = None
        self.calib_file = calib_file
        self.port = port
        self.baud = baud
        # Calibration state
        self.calib_loaded = False
        self.A = None
        self.bx = self.by = self.bz = 0

        # Try to load calibration automatically
        self._load_calibration()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud)
            print(f"[INFO] Connected to {self.port} at {self.baud}")
            return True
        except Exception as e:
            print(f"[ERROR] Serial connection failed: {e}")
            return False
        
    # -------------------------------------------------------------
    #  LOAD CALIBRATION (called once)
    # -------------------------------------------------------------
    def _load_calibration(self):
        if not os.path.exists(self.calib_file):
            print("!! Calibration file not found → RAW yaw only")
            return

        try:
            with open(self.calib_file, "r") as f:
                calib = json.load(f)

            # Hard-iron
            self.bx, self.by, self.bz = calib["hard_iron_bias"]

            # Soft-iron matrix
            self.A = np.array(calib["soft_iron_matrix"])

            print("Calibration loaded:")
            print("Hard-iron:", self.bx, self.by, self.bz)
            print("Soft-iron matrix:\n", self.A)

            self.calib_loaded = True

        except Exception as e:
            print("!! Failed to load calibration:", e)
            self.calib_loaded = False

    # -------------------------------------------------------------
    #  COMPUTE CALIBRATED YAW
    # -------------------------------------------------------------
    def _compute_yaw(self, mx, my, mz):
        if not self.calib_loaded:
            return None

        # Hard-iron
        t = np.array([mx - self.bx, my - self.by, mz - self.bz])

        # Soft-iron
        calibrated = self.A @ t
        mx_c, my_c, mz_c = calibrated

        yaw = math.degrees(math.atan2(-mx_c, my_c))
        if yaw < 0:
            yaw += 360
        return yaw

    # -------------------------------------------------------------
    #  READ ONE PACKET → RETURN DICT
    # -------------------------------------------------------------
    def read_data(self):
        """Read one full packet and return dictionary of values."""

        # Wait for START header
        if self.ser.read(5) != self.START:
            return None

        packet = self.ser.read(self.SIZE)
        if len(packet) != self.SIZE:
            return None

        lat, lon, yaw_raw, sat, mx, my, mz = struct.unpack(
            self.STRUCT_FORMAT, packet
        )

        yaw_cal = self._compute_yaw(mx, my, mz)

        return {
            "latitude": lat,
            "longitude": lon,
            "satellite_count": sat,
            "yaw_raw": yaw_raw,
            "heading": yaw_cal,
            "mx": mx,
            "my": my,
            "mz": mz,
            'degrees0': 0,
            'degrees1':0,
            'volts0':0 ,
            'amps0':0 ,
            'watts0':0 ,
            'volts1':0 ,
            'amps1':0 ,
            'watts1':0 ,
            'timestamp': str(int(time.time())),
        }

