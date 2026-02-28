from modular_reader import SensorReaderGPS
import time

reader = SensorReaderGPS()
reader.connect()
PRINT_INTERVAL = 0.1   # 10 Hz
last_print = 0.0

while True:
    data = reader.read_data()      # Read as fast as packets arrive
    if not data:
        continue

    now = time.monotonic()
    if now - last_print >= PRINT_INTERVAL:
        print(data)
        last_print = now
