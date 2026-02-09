import serial
import struct
import time

class SerialInterface:
    def __init__(self, port, baud_rate):
        self.ser = serial.Serial(port, baud_rate, timeout=0.1)
        # Input to Firmware: Header(uint16), Current(3), Gyro(3), Mag(3), Quat(4), Kbdot(1), Kp(1), Kd(1), Flags(1), Pad(3)
        self.struct_fmt_in = "<H3f3f3f4ffffB3x" 
        # Output from Firmware: Voltage(3), Mode(1), Padding(3) = 16 bytes
        self.struct_fmt_out = "<3fB3x"      
        self.sync_byte_1 = b'\xb5'
        self.sync_byte_2 = b'\x62'

    def send_packet(self, current_xyz, gyro, mag, quat, k_bdot, kp, kd, debug_flags):
        # Flatten data
        # Sync Header 0x62B5 (B5 then 62 on Little Endian)
        data = [0x62B5] + list(current_xyz) + list(gyro) + list(mag) + list(quat) + [k_bdot, kp, kd, debug_flags]
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
                    # Payload: 3 Voltages + 1 Debug = 4 floats = 16 bytes
                    payload = self.ser.read(16) 
                    if len(payload) == 16:
                        return struct.unpack(self.struct_fmt_out, payload)
        return None

    def close(self):
        self.ser.close()
