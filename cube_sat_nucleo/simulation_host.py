import serial
import struct
import time
import math
import sys

# --- Configuration ---
SERIAL_PORT = "/dev/tty.usbmodem1103" # Adjust this to your Nucleo's port
BAUD_RATE = 115200
DT = 0.01  # Physics step size (10ms) - Sending at 100Hz (Nucleo runs at 1kHz, we can send slower or same speed)
           # Note: Nucleo expects packets. We should try to feed it reasonably fast.

# --- Physics Constants ---
RESISTANCE = 25.0 # Ohms (Simulated Coil)
INDUCTANCE = 0.05 # Henry (Simulated Coil)

# --- Protocol Structs (Must match main.h) ---
# Input to Nucleo: Current(f), Gyro[3](f), Mag[3](f), Quat[4](f)
# Total: 1+3+3+4 = 11 floats = 44 bytes
STRUCT_FMT_IN = "<f3f3f4f" # Little-endian

# Output from Nucleo: Voltage(f), Debug(f)
# Total: 2 floats = 8 bytes
STRUCT_FMT_OUT = "<ff"

def main():
    print(f"Opening Serial Port {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"Error opening port: {e}")
        print("Please check your port name in the script.")
        return

    print("Connected! Starting HITL Simulation...")
    print("Sending Fake Sensor Data -> Nucleo")
    print("Receiving Control Voltage  <- Nucleo")

    # Simulation State
    sim_current = 0.0
    sim_voltage = 0.0 # Applied by Nucleo
    sim_time = 0.0
    
    try:
        while True:
            # 1. Physics Step
            # Simple LR Circuit Model: V = IR + L(dI/dt) => dI/dt = (V - IR) / L
            di_dt = (sim_voltage - (sim_current * RESISTANCE)) / INDUCTANCE
            sim_current += di_dt * DT
            
            # Simulated IMU (Just spinning about Z axis)
            omega_z = 0.1 # rad/s
            angle = omega_z * sim_time
            
            fake_gyro = (0.0, 0.0, omega_z)
            fake_mag  = (20.0, 0.0, 40.0) # Gauss/uT constant
            # Quaternion for Z rotation
            # q = [cos(a/2), 0, 0, sin(a/2)]
            fake_quat = (math.cos(angle/2), 0.0, 0.0, math.sin(angle/2))

            # 2. Pack Data
            # Format: Current, Gyro(3), Mag(3), Quat(4)
            # Flatten tuples
            data = [sim_current] + list(fake_gyro) + list(fake_mag) + list(fake_quat)
            packet = struct.pack(STRUCT_FMT_IN, *data)
            
            # 3. Send
            ser.write(packet)
            
            # 4. Receive Response (Blocking read for sync, or poll)
            # Nucleo replies with 8 bytes
            response = ser.read(8)
            if len(response) == 8:
                sim_voltage, debug_val = struct.unpack(STRUCT_FMT_OUT, response)
            
            # 5. Visualization (Teleplot)
            # Print in Teleplot format: ">var:value\n"
            # We print SimCurrent vs Target (we don't know Nucleo's target here easily unless we infer or it sends it)
            # But we can see what the requested Voltage is.
            # And we can plot our Simulated Current.
            print(f">SimCurrent:{sim_current*1000:.2f}") # mA
            print(f">CmdVoltage:{sim_voltage:.2f}")
            
            # Check debug flag
            # print(f">Debug:{debug_val}") 

            sim_time += DT
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nStopping Simulation...")
        ser.close()

if __name__ == "__main__":
    main()
