
import serial
# import pyudev
import os
import subprocess

class ZoomController:
    def __init__(self, baudrate=115200, timeout=1):
        self.baudrate = baudrate
        self.timeout = timeout

    def set_device_permissions(self,serial_top,serial_static):
        devices = [serial_top, serial_static]
        for dev in devices:
            if os.path.exists(dev):
                try:
                    # Use echo to pipe password to sudo
                    cmd = f"echo '123' | sudo -S chmod 666 {dev}"
                    subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print(f'Done setting permissions')
                except Exception as e:
                    print(f"[ERROR] Failed to set permission on {dev}: {e}")
    
    def _send_command(self, device, command):
        try:
            with serial.Serial(device, baudrate=self.baudrate, timeout=self.timeout) as ser:
                ser.write(bytes.fromhex(command))
                response = ser.read(10)
                # print(f"[{device}] Response: {response.hex()}")
        except Exception as e:
            print(f"[ERROR] Failed to send command to {device}: {e}")


    def setZoom(self,zoom_value,serial_top,serial_static):
        done=False
        zoom_str=str(zoom_value)
        if(zoom_str is None or len(zoom_str)==0):
            return
        print(f'Update Zoom -> Current Zoom Level={zoom_value}')
        zoom_levels = {
            "1":  "81 01 04 47 00 00 00 00 FF",
            "2":  "81 01 04 47 01 06 0A 01 FF",
            "3":  "81 01 04 47 02 00 06 03 FF",
            "4":  "81 01 04 47 02 06 02 08 FF",  # Fixed zoom command for level 4
            "5":  "81 01 04 47 02 0A 01 0D FF",
            "6":  "81 01 04 47 02 0D 01 03 FF",
            "7":  "81 01 04 47 02 0F 06 0D FF",
            "8":  "81 01 04 47 03 01 06 01 FF",
            "9":  "81 01 04 47 03 03 00 0D FF",
            "10": "81 01 04 47 03 04 08 06 FF",
            "11": "81 01 04 47 03 05 0D 07 FF",
            "12": "81 01 04 47 03 07 00 09 FF",
            "13": "81 01 04 47 03 08 02 00 FF",
            "14": "81 01 04 47 03 09 02 00 FF",
            "15": "81 01 04 47 03 0A 00 0A FF",
            "16": "81 01 04 47 03 0A 0D 0D FF",
            "17": "81 01 04 47 03 0B 09 0C FF",
            "18": "81 01 04 47 03 0C 04 06 FF",
            "19": "81 01 04 47 03 0C 0D 0C FF",
            "20": "81 01 04 47 03 0D 06 00 FF",
            "21": "81 01 04 47 03 0D 0D 04 FF",
            "22": "81 01 04 47 03 0E 03 09 FF",
            "23": "81 01 04 47 03 0E 09 00 FF",
            "24": "81 01 04 47 03 0E 0D 0C FF",
            "25": "81 01 04 47 03 0F 01 0E FF",
            "26": "81 01 04 47 03 0F 05 07 FF",
            "27": "81 01 04 47 03 0F 08 0A FF",
            "28": "81 01 04 47 03 0F 0B 06 FF",
            "29": "81 01 04 47 03 0F 0D 0C FF",
            "30": "81 01 04 47 04 00 00 00 FF"
        }
        try:
            command = zoom_levels.get(str(zoom_str))
            if not command:
                print(f"[ERROR] Invalid zoom level: {zoom_str}")
                return
            print(f"[Zoom] Sending level {zoom_str}")

            # Assuming /dev/ttyACM0 is left and /dev/ttyACM1 is right camera        
            self._send_command(serial_top, command)
            self._send_command(serial_static, command)
            done=True                                  
        except Exception as e:
            print(f"Error in Visca command: {e}")
        return done
    
    def enable_image_stabilization(self,serial_top,serial_static):
        # ON ----> 81 01 04 34 02 FF
        # OFF ----> 81 01 04 34 03 FF
        try:
            command = "81 01 04 34 02 FF"
            self._send_command(serial_top, command)
            self._send_command(serial_static, command)
        except Exception as e:
            print(f"Error in Visca command: {e}")
    
    def disable_image_stabilization(self,serial_top,serial_static):
        try:
            command = "81 01 04 34 03 FF"
            self._send_command(serial_top, command)
            self._send_command(serial_static, command)
        except Exception as e:
            print(f"Error in Visca command: {e}")
    
    def toggle_autofocus(self,serial_top,serial_static):
        try:
            # 8x 01 04 38 02 FF just on
            # "81 01 04 38 10 FF" toggler
            command = "81 01 04 38 02 FF"
            self._send_command(serial_top, command)
            self._send_command(serial_static, command)
        except Exception as e:
            print(f"Error in Visca command: {e}")

    def manual_focus(self,serial_top,serial_static):
        try:
            command = "81 01 04 38 03 FF"
            self._send_command(serial_top, command)
            self._send_command(serial_static, command)
        except Exception as e:
            print(f"Error in Visca command: {e}")

    def turn_on_digital_zoom(self,serial_top,serial_static):
        try:
            command = "81 01 04 06 02 FF"
            seperate_command = "81 01 04 36 01 FF"
            self._send_command(serial_top, seperate_command)
            self._send_command(serial_static, seperate_command)
        except Exception as e:
            print(f"Error in Visca command: {e}")
    
    def turn_off_digital_zoom(self,serial_top,serial_static):
        try:
            command = "81 01 04 06 03 FF"
            self._send_command(serial_top, command)
            self._send_command(serial_static, command)
        except Exception as e:
            print(f"Error in Visca command: {e}")

    def digital_zoom(self,zoom_value,serial_top,serial_static):
        try:
            zoom_str=str(zoom_value)
            if(zoom_str is None or len(zoom_str)==0):
                return
            print(f'Update Digital Zoom -> Current Digital Zoom Level={zoom_value}')
            zoom_levels = {
                "1" : "81 01 04 46 00 00 00 00 FF",
                "60" : "81 01 04 46 00 00 08 00 FF",
                "90" : "81 01 04 46 00 00 0A 0A FF",
                "120" : "81 01 04 46 00 00 0C 00 FF",
                "150" : "81 01 04 46 00 00 0C 0C FF",
                "180" : "81 01 04 46 00 00 0D 05 FF",
                "210" : "81 01 04 46 00 00 0D 0B FF",
                "240" : "81 01 04 46 00 00 0E 00 FF",
                "270" : "81 01 04 46 00 00 0E 03 FF",
                "300" : "81 01 04 46 00 00 0E 06 FF",
                "330" : "81 01 04 46 00 00 0E 08 FF",
                "360" : "81 01 04 46 00 00 0E 0B FF"
            }
            command = zoom_levels[str(zoom_value)]  # Convert zoom_value back to string for dictionary lookup
            self._send_command(serial_top, command)
            self._send_command(serial_static, command)
        except Exception as e:
            print(f"Error in Visca command: {e}")

