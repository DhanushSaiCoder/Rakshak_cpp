import struct
import serial
import time
import Globals as G
import csv
import os
import queue
import threading
import gzip
import shutil
from MQ_Base import BaseMQ_New
import Globals

# --- Logger Configuration ---
MAX_LOG_FILE_SIZE_MB = 100
MAX_TOTAL_DIR_SIZE_MB_WARNING = 10
MAX_TOTAL_DIR_SIZE_MB_STOP = 15
MAX_LOG_FILE_SIZE_BYTES = MAX_LOG_FILE_SIZE_MB * 1024 * 1024
MAX_TOTAL_DIR_SIZE_BYTES_WARNING = MAX_TOTAL_DIR_SIZE_MB_WARNING * 1024 * 1024 * 1024
MAX_TOTAL_DIR_SIZE_BYTES_STOP = MAX_TOTAL_DIR_SIZE_MB_STOP * 1024 * 1024 * 1024
MAX_UNCOMPRESSED_FILES = 10

class SensorReader:
    def __init__(self):
        self.ser = None
        self.buffer = b''
        self.cnt = 0
        self.t1 = time.time()
        self.data_size = struct.calcsize(G.DATA_FORMAT)
        # Offsets for calibration
        self.calibration_offset = {
            'degrees0': 0.0,
            'degrees1': 0.0
        }

        # Store the most recent raw values
        self._last_raw_degrees0 = 0.0
        self._last_raw_degrees1 = 0.0
        self.mq = BaseMQ_New(queue=f"{Globals.SYSTEM_ID}.pub.q", connection_name=f"{Globals.SYSTEM_ID}-pub")

    def connect(self):
        try:
            self.ser = serial.Serial(G.SENSORS_PORT, G.BAUD_RATE)
            print(f"[INFO] Connected to {G.SENSORS_PORT} at {G.BAUD_RATE}")
            return True
        except Exception as e:
            print(f"[ERROR] Serial connection failed: {e}")
            payload = {"connected":False,"message":"sensors_arduino not connected"}
            self.mq.publish("error","error",payload)
            return False


    def calibrate(self):
        """Use the most recent raw values as the new zero reference"""
        self.calibration_offset['degrees0'] = self._last_raw_degrees0
        self.calibration_offset['degrees1'] = self._last_raw_degrees1
        print(f"[CALIBRATE] Zeroed encoders: {self.calibration_offset}")

    def read_data(self):
        try:
            byte = self.ser.read(1)
            self.buffer += byte
            if len(self.buffer) > len(G.SYNC_WORD):
                self.buffer = self.buffer[-len(G.SYNC_WORD):]

            if self.buffer == G.SYNC_WORD:
                self.cnt += 1
                payload = self.ser.read(self.data_size)
                unpacked = struct.unpack(G.DATA_FORMAT, payload)
                
                # Save raw values (before calibration) for possible recalibration
                self._last_raw_degrees0 = unpacked[0] * 360.0 / 4096.0
                self._last_raw_degrees1 = unpacked[1] * 360.0 / 4096.0

                # Apply calibration offsets
                deg0 = (self._last_raw_degrees0 - self.calibration_offset['degrees0']) % 360.0
                deg1 = (self._last_raw_degrees1 - self.calibration_offset['degrees1']) % 360.0

                data = {
                    'degrees0': deg0,
                    'degrees1':deg1,
                    'volts0': unpacked[2],
                    'amps0': unpacked[3],
                    'watts0': unpacked[4],
                    'volts1': unpacked[5],
                    'amps1': unpacked[6],
                    'watts1': unpacked[7],
                    'satellite_count': unpacked[8],
                    'heading': unpacked[9],
                    'latitude': unpacked[10],
                    'longitude': unpacked[11],
                    'timestamp': str(int(time.time())),
                }

                if time.time() - self.t1 >= 1:
                    self.t1 = time.time()
                    self.cnt = 0

                return data
            return None
        except Exception as e:
            print(f"[ERROR] Serial connection failed: {e}")
            payload = {"connected":False,"message":"sensors_arduino disconnected"}
            self.mq.publish("error","error",payload)
            return False

class SensorLogger:
    def __init__(self, log_dir="sensor_logs", max_file_size=MAX_LOG_FILE_SIZE_BYTES,
                 max_dir_size_warning=MAX_TOTAL_DIR_SIZE_BYTES_WARNING,
                 max_dir_size_stop=MAX_TOTAL_DIR_SIZE_BYTES_STOP,
                 max_uncompressed_files=MAX_UNCOMPRESSED_FILES, queue_size=10000):
        """Initialize logger with configurable parameters."""
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        self.max_file_size = max_file_size
        self.max_dir_size_warning = max_dir_size_warning
        self.max_dir_size_stop = max_dir_size_stop
        self.max_uncompressed_files = max_uncompressed_files
        self.log_index = 0
        self.file = None
        self.writer = None
        self.size = 0
        self.queue = queue.Queue(maxsize=queue_size)
        self.thread = None
        self.running = False
        self._open_new_file()
        self._start_thread()

    def _open_new_file(self):
        """Open a new CSV file and write headers."""
        if self.file:
            self.file.close()
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        filename = os.path.join(self.log_dir, f"sensor_log_{timestamp}_{self.log_index}.csv")
        self.file = open(filename, "w", newline='', buffering=8192)
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "timestamp", "degrees0", "degrees1",
            "volts0", "amps0", "watts0",
            "volts1", "amps1", "watts1",
            "satellite_count", "heading", "latitude", "longitude"
        ])
        self.size = self.file.tell()
        # print(f"[Logger] Logging to {filename}")

    def _get_dir_size(self):
        """Calculate total directory size (bytes)."""
        total = 0
        for root, _, files in os.walk(self.log_dir):
            for f in files:
                total += os.path.getsize(os.path.join(root, f))
        return total

    def _compress_file(self, file_path):
        """Compress a file to .gz and remove original."""
        try:
            gz_path = file_path + '.gz'
            with open(file_path, 'rb') as f_in:
                with gzip.open(gz_path, 'wb') as f_out:
                    shutil.copyfileobj(f_in, f_out)
            os.remove(file_path)
            # print(f"[Logger] Compressed {file_path} to {gz_path}")
        except Exception as e:
            print(f"[ERROR] Compression failed for {file_path}: {e}")

    def _manage_logs(self):
        """Enforce max uncompressed files and dir size."""
        csv_files = [os.path.join(self.log_dir, f) for f in os.listdir(self.log_dir) if f.endswith('.csv')]
        csv_files.sort(key=os.path.getmtime, reverse=True)
        for old_file in csv_files[self.max_uncompressed_files:]:
            self._compress_file(old_file)

        dir_size = self._get_dir_size()
        if dir_size >= self.max_dir_size_warning:
            print(f"[WARNING] {self.log_dir} exceeds {self.max_dir_size_warning // (1024 * 1024)} MB!")
        if dir_size >= self.max_dir_size_stop:
            gz_files = [os.path.join(self.log_dir, f) for f in os.listdir(self.log_dir) if f.endswith('.gz')]
            gz_files.sort(key=os.path.getmtime)
            for old_gz in gz_files:
                os.remove(old_gz)
                print(f"[Logger] Deleted {old_gz} to free space.")
                dir_size -= os.path.getsize(old_gz)
                if dir_size < self.max_dir_size_stop:
                    break
            if dir_size >= self.max_dir_size_stop:
                raise Exception(f"[CRITICAL] Logging stopped: {self.log_dir} exceeds {self.max_dir_size_stop // (1024 * 1024)} MB.")

    def _logger_thread(self):
        """Thread to process queue and write logs."""
        while self.running:
            try:
                row = self.queue.get(timeout=1.0)  # Timeout to check running flag
                if row is None or isinstance(row, bool) :
                    break

                row_bytes = sum(len(str(val)) + 1 for val in row.values())
                if self.size + row_bytes > self.max_file_size:
                    self.log_index += 1
                    self._open_new_file()
                    self._manage_logs()

                self.writer.writerow([
                    row["timestamp"], row["degrees0"], row["degrees1"],
                    row["volts0"], row["amps0"], row["watts0"],
                    row["volts1"], row["amps1"], row["watts1"],
                    row["satellite_count"], row["heading"],
                    row["latitude"], row["longitude"]
                ])
                self.size += row_bytes
                self.queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[ERROR] Logger thread error: {e}")
        self.file.close()
        print("[Logger] Thread stopped, final file closed.")

    def _start_thread(self):
        """Start the logging thread."""
        self.running = True
        self.thread = threading.Thread(target=self._logger_thread, daemon=True)
        self.thread.start()

    def log(self, data):
        """Add data to the queue for logging."""
        if self.running:
            try:
                self.queue.put(data, block=False)
            except queue.Full:
                print("[WARNING] Log queue full, dropping data!")
        else:
            print("[ERROR] Logger not running, cannot log data.")

    def stop(self):
        """Stop the logger, flush queue, and close files."""
        self.running = False
        self.queue.put(None)  # Signal thread to stop
        self.queue.join()
        self.thread.join()
        print("[Logger] Stopped gracefully.")
