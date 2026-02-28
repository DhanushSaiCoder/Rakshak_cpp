#!/usr/bin/env python3

import serial
import struct
import csv
import numpy as np
import pandas as pd
import json
from scipy import linalg


# ============================================================
# MAGNETOMETER CALIBRATION CLASS
# ============================================================
class MagnetometerCalibrator:
    """Magnetometer calibration using ellipsoid fitting method."""

    def __init__(self, magnetic_field_strength=1000):
        self.F = magnetic_field_strength
        self.b = np.zeros([3, 1])      # Hard iron bias
        self.A_1 = np.eye(3)           # Soft iron correction matrix

    def calibrate(self, data):
        print(f"\nRunning ellipsoid calibration on {len(data)} samples...")

        s = data.T
        M, n, d = self._ellipsoid_fit(s)

        M_inv = linalg.inv(M)
        self.b = -np.dot(M_inv, n)

        self.A_1 = np.real(
            self.F / np.sqrt(np.dot(n.T, np.dot(M_inv, n)) - d)
            * linalg.sqrtm(M)
        )

        print("\n=== HARD IRON OFFSET ===")
        print(self.b.flatten())

        print("\n=== SOFT IRON MATRIX ===")
        print(self.A_1)

    def apply(self, data):
        """Apply hard + soft iron correction."""
        return (data - self.b.T) @ self.A_1.T

    def save_json(self, filename):
        cal = {
            "hard_iron_bias": self.b.flatten().tolist(),
            "soft_iron_matrix": self.A_1.tolist(),
            "magnetic_field_strength": self.F
        }
        with open(filename, "w") as f:
            json.dump(cal, f, indent=4)
        print(f"Saved calibration file: {filename}")

    def _ellipsoid_fit(self, s):
        """Least squares ellipsoid fitting (Li & Griffiths method)."""
        D = np.array([
            s[0] ** 2., s[1] ** 2., s[2] ** 2.,
            2. * s[1] * s[2], 2. * s[0] * s[2], 2. * s[0] * s[1],
            2. * s[0], 2. * s[1], 2. * s[2], np.ones_like(s[0])
        ])

        S = np.dot(D, D.T)
        S11 = S[:6, :6]
        S12 = S[:6, 6:]
        S21 = S[6:, :6]
        S22 = S[6:, 6:]

        C = np.array([
            [-1, 1, 1, 0, 0, 0],
            [1, -1, 1, 0, 0, 0],
            [1, 1, -1, 0, 0, 0],
            [0, 0, 0, -4, 0, 0],
            [0, 0, 0, 0, -4, 0],
            [0, 0, 0, 0, 0, -4],
        ])

        E = np.dot(linalg.inv(C), (S11 - np.dot(S12, np.dot(linalg.inv(S22), S21))))

        eigenvalues, eigenvectors = np.linalg.eig(E)
        v1 = eigenvectors[:, np.argmax(eigenvalues)]
        if v1[0] < 0:
            v1 = -v1

        v2 = -np.dot(np.dot(linalg.inv(S22), S21), v1)

        M = np.array([
            [v1[0], v1[5], v1[4]],
            [v1[5], v1[1], v1[3]],
            [v1[4], v1[3], v1[2]],
        ])
        n = np.array([[v2[0]], [v2[1]], [v2[2]]])
        d = v2[3]

        return M, n, d


# ============================================================
#      SERIAL DATA COLLECTION (500 SAMPLES)
# ============================================================
def collect_samples():

    ser = serial.Serial("/dev/ttyUSB0", 250000)
    START = b"START"
    STRUCT_FORMAT = "<fffhfff"   # lat, lon, yaw, sat, mx, my, mz
    SIZE = struct.calcsize(STRUCT_FORMAT)

    MAX_SAMPLES = 500
    samples = []

    print("Collecting 500 samples... Rotate sensor slowly in all directions.")

    while len(samples) < MAX_SAMPLES:

        if ser.read(5) != START:
            continue

        data = ser.read(SIZE)
        _, _, _, _, mx, my, mz = struct.unpack(STRUCT_FORMAT, data)

        samples.append([mx, my, mz])
        print(f"{len(samples)}/500  mx={mx:.2f}, my={my:.2f}, mz={mz:.2f}")

    ser.close()
    return np.array(samples)


# ============================================================
# MAIN: COLLECT + CALIBRATE
# ============================================================
def main():

    # 1) Collect raw magnetometer samples
    samples = collect_samples()

    # Save raw CSV
    raw_csv = "mag_500.csv"
    pd.DataFrame(samples, columns=["mx", "my", "mz"]).to_csv(raw_csv, index=False)
    print(f"\nSaved raw data to: {raw_csv}")

    # 2) Calibrate using ellipsoid fitting
    calib = MagnetometerCalibrator()
    calib.calibrate(samples)

    # 3) Save JSON calibration file
    calib.save_json("mag_500_calibration.json")

    # 4) Apply calibration to all samples
    corrected = calib.apply(samples)

    # Save calibrated CSV
    pd.DataFrame(corrected, columns=["mx_cal", "my_cal", "mz_cal"]).to_csv(
        "mag_500_calibrated.csv", index=False
    )
    print("Saved calibrated values to: mag_500_calibrated.csv")

    print("\n DONE! Calibration completed successfully.\n")


if __name__ == "__main__":
    main()

