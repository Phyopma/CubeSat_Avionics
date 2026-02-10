"""
ADCS Simulation Host v3.0 - HITL Only
Runs Hardware-In-The-Loop simulation with real STM32 firmware.
No mock firmware - always requires physical hardware connection.
"""
import struct
import time
import math
import sys
import argparse
import numpy as np

from physics import SatellitePhysics
from comms import SerialInterface
from telemetry import TelemetryManager, VisualizerSender

# --- Configuration ---
SERIAL_PORT = "/dev/cu.usbmodem21303"  # Default Nucleo port
BAUD_RATE = 115200
DT = 0.01  # Physics step size = 10ms (100Hz) - Default - Default - Default
TELEPLOT_ADDR = ("teleplot.fr", 28011)

def get_args():
    parser = argparse.ArgumentParser(description='ADCS Simulation Host (HITL Only)')
    parser.add_argument('--scenario', choices=['detumble', 'pointing'], default='detumble',
                        help='Simulation scenario')
    parser.add_argument('--initial_omega', nargs=3, type=float, metavar=('WX', 'WY', 'WZ'),
                        help='Initial angular velocity [rad/s]')
    parser.add_argument('--duration', type=float, default=0,
                        help='Simulation duration in seconds (0 = infinite)')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress console output')
    parser.add_argument('--realtime', action='store_true',
                        help='Run simulation at 1x speed (90min orbit)')
    parser.add_argument('--open-loop', action='store_true',
                        help='Bypass PI controller on firmware (V=I*R)')
    
    # Controller Gains (sent to firmware for runtime tuning)
    parser.add_argument('--kbdot', type=float, default=1000000.0,
                        help='B-dot gain for detumble mode')
    parser.add_argument('--kp', type=float, default=0.100,
                        help='Proportional gain for pointing mode')
    parser.add_argument('--ki', type=float, default=0.000,
                        help='Integral gain for pointing mode')
    parser.add_argument('--kd', type=float, default=0.100,
                        help='Derivative gain for pointing mode')
    
    # Hardware
    parser.add_argument('--port', type=str, default=SERIAL_PORT,
                        help='Serial port for Nucleo board')
    
    return parser.parse_args()


def main():
    args = get_args()
    
    if not args.quiet:
        print("--- ADCS Simulation Host v3.0 (HITL) ---")
        print(f"Port: {args.port}")
        print(f"Gains: K_BDOT={args.kbdot:.0e}, K_P={args.kp}, K_I={args.ki}, K_D={args.kd}")
    
    # Initialize components
    sat = SatellitePhysics()
    tel = TelemetryManager(TELEPLOT_ADDR)
    viz = VisualizerSender()
    
    # Set initial conditions based on scenario
    if args.initial_omega:
        w_init = np.array(args.initial_omega)
    elif args.scenario == 'detumble':
        w_init = np.array([0.5, 0.5, 0.5])  # High tumble for detumble test
    else:  # pointing
        w_init = np.array([0.05, 0.05, 0.05])  # Low tumble for pointing
    sat.state[4:7] = w_init
    
    if not args.quiet:
        print(f"Scenario: {args.scenario}, Initial ω: {w_init}")
    
    # Connect to firmware
    try:
        comms = SerialInterface(args.port, BAUD_RATE)
    except Exception as e:
        print(f"ERROR: Cannot open port {args.port}: {e}")
        print("Make sure the Nucleo board is connected and the port is correct.")
        sys.exit(1)
    
    if not args.quiet:
        print("Connected to firmware. Starting simulation...")
    
    # Simulation state
    sim_time = 0.0
    torque_applied = np.array([0.0, 0.0, 0.0])
    sat_currents = np.array([0.0, 0.0, 0.0])
    firmware_voltages = np.array([0.0, 0.0, 0.0])
    adcs_mode_id = 0
    
    try:
        while True:
            # Check termination condition
            if args.duration > 0 and sim_time >= args.duration:
                q = sat.state[0:4]
                w = sat.state[4:7]
                qw, qx, qy, qz = q
                dot = max(-1.0, min(1.0, 1 - 2*(qx**2 + qy**2)))
                angle_err = math.degrees(math.acos(dot))
                print(f"FINAL_STATE: W={np.linalg.norm(w):.6f} rad/s | Err={angle_err:.6f} deg")
                break
            
            # Time step
            # Realtime: 1.0s physics step (slow but precise enough for 90min)
            # Fast: 0.1s physics step (10x speedup relative to orbit dynamics)
            if args.realtime:
                DT = 1.0 
                if int(sim_time) == 0: print("Running in REALTIME mode (1s step, 90min orbit)")
            else:
                DT = 0.1
                if int(sim_time) == 0: print("Running in FAST mode (0.1s step, ~10 mins per orbit)")
            
            t_start = time.perf_counter()
            
            # Get environment state
            B_body = sat.get_magnetic_field(sim_time)
            
            # Step physics forward
            sat.rk4_step(sim_time, DT, firmware_voltages, B_body)
            
            q = sat.state[0:4]
            w = sat.state[4:7]
            sat_currents = sat.state[7:10]
            
            # Current estimate for firmware (using physics state)
            estimated_current = sat_currents.tolist()
            
            # Send sensor data to firmware, receive commands
            debug_flags = 1 if args.open_loop else 0
            comms.send_packet(estimated_current, w, B_body, q, args.kbdot, args.kp, args.ki, args.kd, debug_flags)
            response = comms.read_packet()
            
            if response:
                firmware_voltages = np.array(response[0:3])
                adcs_mode_id = response[3]
            else:
                # No response - firmware may be busy, keep last values
                pass
            
            # --- Telemetry (10Hz) ---
            if not args.quiet and int(sim_time / DT) % 10 == 0:
                # Calculate torque for telemetry based on physics state current
                m_actual = sat_currents * 5.76 # Stronger MTQ Factor
                torque_applied = np.cross(m_actual, B_body)
            if not args.quiet and int(sim_time / DT) % 10 == 0:
                qw, qx, qy, qz = q
                dot = max(-1.0, min(1.0, 1 - 2*(qx**2 + qy**2)))
                angle_err = math.degrees(math.acos(dot))
                m_str = tel.mode_map.get(adcs_mode_id, str(adcs_mode_id))
                
                # Send to Teleplot
                i_target = sat_currents  # Use actual current as target estimate
                tel.send_telemetry(w, q, torque_applied, sim_time, adcs_mode_id, i_target, sat_currents, firmware_voltages)
                
                # 3D Visualization
                tel.send_3d_cube("Body,3D", q, "#3498db", pos=(0,0,0), dims=(1,1,2))
                tel.send_3d_vector("Target,3D", (0,0,0), np.array([0,0,1]), "#2ecc71", scale=2.5, thickness=0.08)
                
                # Body Z-axis in inertial frame
                q0, q1, q2, q3 = q
                body_z_inertial = np.array([
                    2*(q1*q3 + q0*q2),
                    2*(q2*q3 - q0*q1),
                    1 - 2*(q1**2 + q2**2)
                ])
                tel.send_3d_vector("BodyZ,3D", (0,0,0), body_z_inertial, "#00ffff", scale=2.5, thickness=0.08)
                
                # Stability check: τ·ω < 0 means damping
                work_rate = np.dot(torque_applied, w)
                stability_str = "DAMPING" if work_rate < 0 else "SPIN-UP"
                
                # Console output (1Hz)
                if int(sim_time / DT) % 100 == 0:
                    print(f"t={sim_time:5.1f}s | Mode={m_str:10s} | W={np.linalg.norm(w):.3f} | Err={angle_err:5.1f}° | {stability_str} (P={work_rate:.2e}) | V={firmware_voltages}")
            
            sim_time += DT
            
            # Real-time pacing
            t_elapsed = time.perf_counter() - t_start
            if not args.quiet:
                time.sleep(max(0, DT - t_elapsed))
    
    except KeyboardInterrupt:
        print("\nSimulation Terminated.")
    finally:
        comms.close()


if __name__ == "__main__":
    main()
