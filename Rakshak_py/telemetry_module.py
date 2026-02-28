import time
import json
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Dict, Any, Tuple, List

import serial  # pip install pyserial

from tegrastats_reader import TegrastatsReader
from MQ_Base import BaseMQ_New

# Same anchor-table approach used in your codebase (pq(hex) -> range)
TEMP_TABLE: List[Tuple[int, Tuple[int, int]]] = [
    (0xFB, (-8, -2)),
    (0x00, (-3,  3)),
    (0x0A, ( 7, 13)),
    (0x14, (17, 23)),
    (0x1E, (27, 33)),
    (0x28, (37, 43)),
    (0x32, (47, 53)),
    (0x3C, (57, 63)),
]

def lookup_temp_range_from_pq(pq: int) -> Optional[Tuple[int, int]]:
    last = None
    for anchor, rng in TEMP_TABLE:
        if pq >= anchor:
            last = rng
    return last


@dataclass
class CamSpec:
    name: str
    port: str           # "/dev/ttyACM0"
    visca_addr: int = 1 # 1..7 typically
    baud: int = 115200


def _read_until_ff(ser: serial.Serial, timeout_s: float = 0.6, max_bytes: int = 32) -> bytes:
    data = bytearray()
    t0 = time.time()
    while len(data) < max_bytes and (time.time() - t0) < timeout_s:
        b = ser.read(1)
        if not b:
            continue
        data += b
        if b[0] == 0xFF:
            break
    return bytes(data)


def read_visca_cam_temp(ser: serial.Serial, visca_addr: int) -> Dict[str, Any]:
    """
    Sends CAM_TempInq: 8x 09 04 68 FF  (x = address)
    Expects response: y0 50 00 00 0p 0q FF
    """
    out: Dict[str, Any] = {
        "pq_hex": None,
        "temp_c": None,
        "raw_converted_pq": None,
        "raw_response_hex": None,
        "err": "",
    }

    addr = visca_addr & 0x0F
    cmd = bytes([0x80 | addr, 0x09, 0x04, 0x68, 0xFF])

    try:
        ser.reset_input_buffer()
        ser.write(cmd)
        ser.flush()

        resp = _read_until_ff(ser, timeout_s=0.6, max_bytes=32)
        out["raw_response_hex"] = resp.hex()

        # Validate minimal format
        if len(resp) < 7 or resp[1] != 0x50:
            out["err"] = "Unexpected response format"
            return out

        p = resp[4] & 0x0F
        q = resp[5] & 0x0F
        pq = (p << 4) | q

        out["raw_converted_pq"] = pq
        out["pq_hex"] = f"0x{pq:02X}"

        rng = lookup_temp_range_from_pq(pq)
        if rng:
            t_min, t_max = rng
            out["temp_c"] = round((t_min + t_max) / 2.0, 1)
        else:
            out["err"] = "pq out of table"

        return out

    except Exception as e:
        out["err"] = f"{type(e).__name__}: {e}"
        return out


def monitor_two_cameras_and_jetson(left: CamSpec, right: CamSpec, hz: float = 1.0):
    """
    Prints dict every second:
      - cpu/gpu temp
      - cpu/gpu usage
      - ram available GB
      - left/right camera temp via VISCA over ttyACM*
    """
    period = 1.0 / max(hz, 0.1)

    # Start tegrastats
    ts = TegrastatsReader(interval_ms=200)
    ts.start()

    # Open serial ports once
    ser_left = serial.Serial(left.port, left.baud, timeout=0.1)
    ser_right = serial.Serial(right.port, right.baud, timeout=0.1)

    # small settle
    time.sleep(0.05)

    next_t = time.perf_counter()
    cnt = 0

    mq_publisher = BaseMQ_New(queue=f"Telemetry.pub.q", connection_name=f"Telemetry-pub")  # queue unused for pub but harmless
    print(f'Telimetry data publishing stated')  
    try:
        while True:
            row: Dict[str, Any] = {
                "cnt": cnt,
                "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }
            cnt += 1

            # Jetson stats
            j = ts.read_one()
            row["cpu_temp_c"] = j.get("cpu_temp_c")
            row["gpu_temp_c"] = j.get("gpu_temp_c")
            row["cpu_usage_pct"] = j.get("cpu_usage_overall_pct")
            row["gpu_usage_pct"] = j.get("gpu_usage_overall_pct")

            # RAM available (tegrastats returns strings like "7021")
            try:
                used_mb = int(j.get("ram_used_mb")) if j.get("ram_used_mb") is not None else None
                total_mb = int(j.get("ram_total_mb")) if j.get("ram_total_mb") is not None else None
                if used_mb is not None and total_mb is not None and total_mb > 0:
                    row["ram_available_gb"] = round((total_mb - used_mb) / 1024.0, 2)
                else:
                    row["ram_available_gb"] = None
            except Exception:
                row["ram_available_gb"] = None

            # Camera temps
            row["cameras"] = {
                left.name:  {"port": left.port,  **read_visca_cam_temp(ser_left,  left.visca_addr)},
                right.name: {"port": right.port, **read_visca_cam_temp(ser_right, right.visca_addr)},
            }

            # print(json.dumps(row, indent=2))
            data = json.dumps(row, indent=2)
            mq_publisher.publish("event","system_stats",{"payload":data})

            # pace
            next_t += period
            sleep_s = next_t - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.perf_counter()

    finally:
        try:
            ts.stop()
        except Exception:
            pass
        try:
            ser_left.close()
        except Exception:
            pass
        try:
            ser_right.close()
        except Exception:
            pass

def telemetry_process(left_name: str, left_port: str,
                      right_name: str, right_port: str,
                      hz: float = 1.0,
                      left_addr: int = 1,
                      right_addr: int = 1):
    """
    Process entrypoint. Keep signature simple & primitive types only.
    """
    left = CamSpec(name=left_name, port=left_port, visca_addr=left_addr)
    right = CamSpec(name=right_name, port=right_port, visca_addr=right_addr)

    # this opens serial + tegrastats INSIDE the process
    monitor_two_cameras_and_jetson(left, right, hz=hz)
