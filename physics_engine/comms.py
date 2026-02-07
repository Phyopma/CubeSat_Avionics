import serial
import struct
import time

class SerialInterface:
    def __init__(self, port, baud_rate):
        self.ser = serial.Serial(port, baud_rate, timeout=0.1)
        self.struct_fmt_in = "<f3f3f4ff" # Current, Gyro[3], Mag[3], Quat[4], TargetCmd
        self.struct_fmt_out = "<ff"      # Voltage, Debug
        self.sync_byte_1 = b'\xb5'
        self.sync_byte_2 = b'\x62'

    def send_packet(self, current, gyro, mag, quat, target_cmd):
        # Flatten data
        # Quat assumed [w, x, y, z] in Python, but need to check firmware expectation
        # Firmware usually expects valid quat. 
        # Sending: Current(1), Gyro(3), Mag(3), Quat(4), Target(1) = 12 floats = 48 bytes
        data = [current] + list(gyro) + list(mag) + list(quat) + [target_cmd]
        packet = struct.pack(self.struct_fmt_in, *data)
        
        # Flush input to ensure we get fresh data response to this packet
        self.ser.reset_input_buffer()
        self.ser.write(packet)

    def read_packet(self):
        # Try to find Sync Header
        # Max scan steps to avoid infinite loop
        for _ in range(50):
            b1 = self.ser.read(1)
            if not b1: return None # Timeout
            
            if b1 == self.sync_byte_1:
                b2 = self.ser.read(1)
                if b2 == self.sync_byte_2:
                    # Header Found, read payload
                    payload = self.ser.read(8) # 2 floats
                    if len(payload) == 8:
                        return struct.unpack(self.struct_fmt_out, payload)
        return None

    def close(self):
        self.ser.close()
