import serial
import time
import Globals
from MQ_Base import BaseMQ_New
# lrwxrwxrwx 1 root root 7 Jul 29 11:39 /dev/arduino_sensors -> ttyUSB1
# lrwxrwxrwx 1 root root 7 Jul 29 11:39 /dev/arduino_trigger -> ttyUSB0

class TriggerController:
    def __init__(self, port=Globals.TRIGGER_PORT, baudrate=115200, timeout=1):
        """Initialize the serial connection to Arduino."""
        # self.mq = BaseMQ_New(queue=f"{Globals.SYSTEM_ID}.pub.q", connection_name=f"{Globals.SYSTEM_ID}-pub")
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Allow Arduino to initialize
        except Exception as e:
            payload = {"connected":False,"message":"trigger_arduino not connected"}
            # self.mq.publish("error","error",payload)
            print(e)

        print(f"Connected to {port} at {baudrate} baud")

    # ------------------- LASER FUNCTIONS ------------------- #
    def laser_on(self):
        self.ser.write("8\n".encode())
        # print("laser ON")

    def laser_off(self):
        self.ser.write("9\n".encode())
        # print("laser OFF")

    # ------------------- SOLENOID FUNCTIONS ------------------- #
    def set_duration(self, duration):
        """Set the solenoid ON duration (ms)."""
        self.ser.write(f"D{duration}\n".encode())
        # print(f"solenoid duration set to {duration} ms")

    def solenoid_on(self):
        self.ser.write("6\n".encode())
        # print("solenoid ON")

    def solenoid_off(self):
        self.ser.write("7\n".encode())
        # print("solenoid OFF")

    def solenoid_on_off(self):
        self.ser.write("5\n".encode())
        # print("solenoid ON then OFF")

    # ------------------- BRAKE FUNCTIONS ------------------- #
    def set_brake_duration(self, duration):
        self.ser.write(f"L{duration}\n".encode())
        # print(f"Brake duration set to {duration} ms")

    def brake_on(self):
        self.ser.write("2\n".encode())
        # print("Brake ON")

    def brake_off(self):
        self.ser.write("3\n".encode())
        # print("Brake OFF")

    def brake_on_off(self):
        self.ser.write("1\n".encode())
        # print("Brake ON then OFF")

    def brake_on_off_sol(self):
        self.ser.write("9\n".encode())
        # print("Brake ON then OFF (Solenoid)")

    # ------------------- SERVO FUNCTION ------------------- #
    def move_servo(self, start_pos, target_angle, delay_time):
        """
        Move the servo from start position to target angle with delay.
        Command format: startPos,angle,delayTime
        """
        command = f"{start_pos},{target_angle},{delay_time}\n"
        self.ser.write(command.encode())
        # print(f"Sent servo command: {command.strip()}")

        # Read Arduino responses if available
        while self.ser.in_waiting:
            response = self.ser.readline().decode().strip()
            # print(f"Arduino: {response}")

    # ------------------- CLEANUP ------------------- #
    def close(self):
        """Close the serial connection."""
        if self.ser.is_open:
            self.ser.close()
            # print("Serial connection closed.")
