import serial
import struct
import time
import math
import sys
import argparse
import random

import socket

# --- Configuration ---
SERIAL_PORT = "/dev/cu.usbmodem21303" # Adjust this to your Nucleo's port
BAUD_RATE = 115200
DT = 0.01  # Physics step size (10ms)
TELEPLOT_ADDR = ("teleplot.fr", 34329) # User-specified Teleplot server

# --- Physics Constants ---
RESISTANCE = 25.0 # Ohms
INDUCTANCE = 0.5 # Henry (Increased for HITL stability with DT=0.01)

# --- Protocol Structs (Must match main.h) ---
# Input to Nucleo: Current(f), Gyro[3](f), Mag[3](f), Quat[4](f), TargetCmd(f)
# Total: 1+3+3+4+1 = 12 floats = 48 bytes
STRUCT_FMT_IN = "<f3f3f4ff" # Added extra 'f' at the end

# Output from Nucleo: Voltage(f), Debug(f)
STRUCT_FMT_OUT = "<ff"

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
    
    print(f"Opening Serial Port {port}...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Streaming Teleplot data to {TELEPLOT_ADDR}")

    print(f"Connected! Mode: {args.mode}, Amp: {args.amp}A, Freq: {args.freq}Hz")
    
    # Simulation State
    sim_current = 0.0
    sim_voltage = 0.0
    sim_time = 0.0
    target_current = 0.0
    
    last_step_time = 0.0
    
    try:
        while True:
            # 1. Waveform Generation (The Setpoint)
            if args.mode == 'step':
                # Toggle every 1/freq seconds (half period)
                period = 1.0 / args.freq
                if (sim_time % period) < (period / 2):
                    target_current = args.amp
                else:
                     target_current = 0.0 # Return to zero or -amp? Let's do 0 to Amp.
            elif args.mode == 'sine':
                target_current = args.amp * math.sin(2 * math.pi * args.freq * sim_time)
            elif args.mode == 'random':
                 period = 1.0 / args.freq # Change target every period
                 if sim_time - last_step_time > period:
                     target_current = random.uniform(-args.amp, args.amp)
                     last_step_time = sim_time
            elif args.mode == 'manual':
                target_current = args.amp # Constant

            # 2. Physics Step (Inner Loop Plant)
            # Model Power Supply Limit (5V Rail)
            sim_voltage = max(min(sim_voltage, 5.0), -5.0)

            # V = IR + L(dI/dt)
            di_dt = (sim_voltage - (sim_current * RESISTANCE)) / INDUCTANCE
            sim_current += di_dt * DT
            
            # Physics Check: Abs Max Current = 5V / 25R = 0.2A
            # Safety Clamp set to 0.5A to handle transients
            sim_current = max(min(sim_current, 0.5), -0.5)
            
            # Simulated IMU (Detailed physics comes later in Phase 2)
            omega_z = 0.1
            angle = omega_z * sim_time
            fake_gyro = (0.0, 0.0, omega_z)
            fake_mag  = (20.0, 0.0, 40.0)
            fake_quat = (math.cos(angle/2), 0.0, 0.0, math.sin(angle/2))

            # 3. Pack Data including new TargetCmd
            data = [sim_current] + list(fake_gyro) + list(fake_mag) + list(fake_quat) + [target_current]
            packet = struct.pack(STRUCT_FMT_IN, *data)
            
            # 4. Send & Receive
            # CRITICAL: Firmware runs at 1kHz, we run at 100Hz.
            # Buffer contains ~10 old packets. Flush them to get Fresh Data.
            ser.reset_input_buffer()
            
            ser.write(packet)
            
            # Read Response with Sync Header (0xB5 0x62)
            # This prevents byte misalignment (reading float from middle of packet)
            scan_count = 0
            MAX_SCAN = 50 # Don't get stuck forever
            while scan_count < MAX_SCAN:
                scan_count +=1
                b1 = ser.read(1)
                if len(b1) == 0: # Timeout
                    break 
                if b1 == b'\xb5':
                    # Read 2nd byte
                    b2 = ser.read(1)
                    if len(b2) == 0: break
                    
                    if b2 == b'\x62':
                        # HEADER FOUND! Read the payload (2 floats = 8 bytes)
                        payload = ser.read(8)
                        if len(payload) == 8:
                            sim_voltage, debug_val = struct.unpack(STRUCT_FMT_OUT, payload)
                            break # Success! (Exit loop to process)
                        else:
                            # Incomplete payload
                            break
            
            # Error Calculation
            sim_error = target_current - sim_current
            
            # NaN Protection / Auto-Reset
            if math.isnan(sim_current) or math.isnan(sim_voltage):
                 print(f"Warning: NaN detected. Resetting simulation state.")
                 sim_current = 0.0
                 sim_voltage = 0.0
                 sim_error = 0.0
            
            # 5. Visualization (UDP to Teleplot)
            # Format: varName:timestamp:value\n
            now_ms = int(time.time() * 1000)
            telemetry = (
                f"Target:{now_ms}:{target_current*1000:.2f}\n"
                f"SimCurrent:{now_ms}:{sim_current*1000:.2f}\n"
                f"Error:{now_ms}:{sim_error*1000:.2f}\n"
                f"CmdVoltage:{now_ms}:{sim_voltage:.2f}\n"
            )
            sock.sendto(telemetry.encode(), TELEPLOT_ADDR)
            
            # Optional: Print status locally every 1s so terminal isn't silent
            if int(sim_time / DT) % 100 == 0:
                 print(f"t={sim_time:.1f}s | Tgt={target_current*1000:.0f}mA | Curr={sim_current*1000:.0f}mA | Err={sim_error*1000:.0f}mA | V={sim_voltage:.1f}V")

            sim_time += DT
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nStopping Simulation...")
        ser.close()

if __name__ == "__main__":
    main()
