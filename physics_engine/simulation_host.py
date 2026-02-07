import struct
import time
import math
import sys
import argparse
import random
import socket
import numpy as np

# Import our new modules
from physics import SatellitePhysics
from comms import SerialInterface

# --- Configuration ---
SERIAL_PORT = "/dev/cu.usbmodem21303" # Adjust this to your Nucleo's port
BAUD_RATE = 115200
DT = 0.01  # Physics step size = 10ms (100Hz)
TELEPLOT_ADDR = ("teleplot.fr", 34329) # User-specified Teleplot server

def get_args():
    parser = argparse.ArgumentParser(description='HITL Simulation Host')
    parser.add_argument('--mode', choices=['step', 'sine', 'random', 'manual'], default='step', help='Waveform mode')
    parser.add_argument('--amp', type=float, default=0.04, help='Amplitude in Amps')
    parser.add_argument('--freq', type=float, default=0.5, help='Frequency in Hz')
    parser.add_argument('--port', type=str, default=SERIAL_PORT, help='Serial port')
    return parser.parse_args()

def main():
    args = get_args()
    port = args.port
    
    print(f"Initializing Physics Engine...")
    sat = SatellitePhysics()
    
    print(f"Opening Serial Port {port}...")
    try:
        comms = SerialInterface(port, BAUD_RATE)
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Streaming Teleplot data to {TELEPLOT_ADDR}")

    print(f"Connected! Mode: {args.mode}, Amp: {args.amp}A, Freq: {args.freq}Hz")
    
    # Simulation State Variables for Telemetry
    sim_time = 0.0
    sim_current = 0.0 # Coil Actuall Current (State)
    target_current = 0.0
    last_step_time = 0.0
    
    # Internal Physics State Integration
    # Torque applied by coils (in Body Frame)
    torque_applied = np.array([0.0, 0.0, 0.0])

    try:
        while True:
            # --- 1. Waveform Logic (The "Reference" Command for Testing) ---
            if args.mode == 'step':
                period = 1.0 / args.freq
                if (sim_time % period) < (period / 2):
                    target_current = args.amp
                else:
                    target_current = 0.0
            elif args.mode == 'sine':
                target_current = args.amp * math.sin(2 * math.pi * args.freq * sim_time)
            elif args.mode == 'random':
                period = 1.0 / args.freq
                if sim_time - last_step_time > period:
                    target_current = random.uniform(-args.amp, args.amp)
                    last_step_time = sim_time
            elif args.mode == 'manual':
                target_current = args.amp

            # --- 2. Physics Step ---
            # Get Environmental B-Field in Body Frame
            B_body = sat.get_magnetic_field(sim_time)
            
            # Mechanical Step
            sat.rk4_step(sim_time, DT, torque_applied)
            
            # Extract state for Serial Packet
            # State: [q0, q1, q2, q3, wx, wy, wz]
            q = sat.state[0:4]
            w = sat.state[4:7]
            
            # --- 3. Communication ---
            # Send Sensor Data to Firmware
            # Packet: Current(1), Gyro(3), Mag(3), Quat(4), TargetCmd(1)
            # We send sim_current as "Measured" so Firmware PI loops on it.
            comms.send_packet(sim_current, w, B_body * 1e6, q, target_current) # Mag in uT? or T?
            # Firmware expects: BNO085 units?
            # Physics B in Tesla.
            # If Firmware print shows "20.5 uT", then we should likely send uT or T depending on `imu_bno085.c`.
            # Let's send T and see. Or consistency with previous `fake_mag = (20.0, ...)` -> probably uT.
            # Multiplying by 1e6.

            # Read Response (Control Effort)
            # Firmware sends: Voltage, Debug
            response = comms.read_packet()
            firmware_voltage = 0.0
            if response:
                firmware_voltage, debug_val = response
                
                # --- Electrical Model (RL Circuit) ---
                # V = IR + L(dI/dt) => dI/dt = (V - IR) / L
                R_coil = 25.0 # Ohms
                L_coil = 0.1  # Henry (Approximation)
                
                di_dt = (firmware_voltage - (sim_current * R_coil)) / L_coil
                sim_current += di_dt * DT
                
                # Compute Torque
                # Assume Z-axis coil
                # Area = 0.01 m^2, Turns = 400? (Guess) -> Eff Area ~ 4
                # m_z = n_turns * current * area
                m_eff = 0.4 # Am^2 per Amp (Guess)
                m_vec = np.array([0.0, 0.0, sim_current * m_eff])
                
                torque_applied = np.cross(m_vec, B_body)
                
            # --- 4. Logging / Teleplot ---
            now_ms = int(time.time() * 1000)
            telemetry = (
                f"Time:{now_ms}:{sim_time:.2f}\n"
                f"TargetA:{now_ms}:{target_current*1000:.2f}\n" 
                f"CmdVolts:{now_ms}:{firmware_voltage:.2f}\n"
                f"OmegaZ:{now_ms}:{w[2]:.4f}\n"
            )
            sock.sendto(telemetry.encode(), TELEPLOT_ADDR)

            # Local Print
            if int(sim_time / DT) % 100 == 0:
                 print(f"t={sim_time:.1f}s | Wz={w[2]:.3f} | V={firmware_voltage:.2f}")

            sim_time += DT
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nStopping Simulation...")
        comms.close()

if __name__ == "__main__":
    main()
