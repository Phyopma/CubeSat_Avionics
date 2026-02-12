"""
ADCS Simulation Host v3.0 - HITL Only
Runs Hardware-In-The-Loop simulation with real STM32 firmware.
No mock firmware - always requires physical hardware connection.
"""
import struct
import time
import math
import sys
import os
import re
import argparse
import numpy as np

from physics import SatellitePhysics
from comms import SerialInterface
from telemetry import TelemetryManager, VisualizerSender

# --- Configuration ---
SERIAL_PORT = "/dev/cu.usbmodem21303"  # Default Nucleo port
BAUD_RATE = 115200
DT = 0.01  # Physics step size = 10ms (100Hz) - Default - Default - Default
TELEPLOT_ADDR = ("teleplot.fr", 45076)
FW_CONFIG_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "cube_sat_nucleo",
        "Application",
        "Algorithms",
        "Inc",
        "config.h",
    )
)
FW_MAX_VOLTAGE_DEFAULT = 3.3
FW_DIPOLE_STRENGTH_DEFAULT = 2.88
HOST_DEFAULT_MAX_VOLTAGE = 3.3
HOST_DEFAULT_DIPOLE_STRENGTH = 2.88
FW_RUNTIME_MAX_VOLTAGE_MIN = 0.5
FW_RUNTIME_MAX_VOLTAGE_MAX = 12.0
FW_RUNTIME_DIPOLE_MIN = 0.1
FW_RUNTIME_DIPOLE_MAX = 20.0


def _extract_define_expr(text, macro):
    m = re.search(rf"^\s*#define\s+{re.escape(macro)}\s+(.+)$", text, re.MULTILINE)
    if not m:
        return None
    return m.group(1).split("//", 1)[0].strip()


def _eval_c_float_expr(expr):
    if not expr:
        return None
    cleaned = re.sub(r"(?<=\d)f\b", "", expr)
    if not re.fullmatch(r"[0-9eE+\-*/().\s]+", cleaned):
        return None
    try:
        return float(eval(cleaned, {"__builtins__": {}}, {}))
    except Exception:
        return None


def read_firmware_calibration():
    fw_max = FW_MAX_VOLTAGE_DEFAULT
    fw_dipole = FW_DIPOLE_STRENGTH_DEFAULT
    source = f"defaults (max={FW_MAX_VOLTAGE_DEFAULT}, dipole={FW_DIPOLE_STRENGTH_DEFAULT})"

    try:
        with open(FW_CONFIG_PATH, "r", encoding="utf-8") as f:
            text = f.read()

        expr_max = _extract_define_expr(text, "PIL_MAX_VOLTAGE")
        expr_mtq = _extract_define_expr(text, "MTQ_DIPOLE_TO_AMP")

        parsed_max = _eval_c_float_expr(expr_max)
        parsed_mtq = _eval_c_float_expr(expr_mtq)

        if parsed_max and parsed_max > 0.0:
            fw_max = parsed_max
        if parsed_mtq and abs(parsed_mtq) > 1e-12:
            fw_dipole = 1.0 / parsed_mtq

        source = FW_CONFIG_PATH
    except OSError:
        pass

    return fw_max, fw_dipole, source

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
    parser.add_argument('--force-mode', choices=['auto', 'detumble', 'spin', 'pointing'], default='auto',
                        help='Force STM32 outer-loop mode (auto keeps firmware state machine)')
    parser.add_argument('--reset-controller', action=argparse.BooleanOptionalAction, default=True,
                        help='Request one-shot outer-loop state reset at run start (default: enabled)')
    parser.add_argument('--dt', type=float,
                        help='Physics step size in seconds (overrides realtime/fast defaults)')
    
    # Controller Gains (sent to firmware for runtime tuning)
    parser.add_argument('--kbdot', type=float, default=400000.0,
                        help='B-dot gain for detumble mode (positive = damping)')
    parser.add_argument('--kp', type=float, default=0.0012,
                        help='Proportional gain for pointing mode')
    parser.add_argument('--ki', type=float, default=3e-05,
                        help='Integral gain for pointing mode')
    parser.add_argument('--kd', type=float, default=0.17,
                        help='Derivative gain for pointing mode')
    
    # Hardware
    parser.add_argument('--port', type=str, default=SERIAL_PORT,
                        help='Serial port for Nucleo board')
    parser.add_argument('--debug', action='store_true',
                        help='Enable detailed physics debug logging')

    # Physics Calibration
    parser.add_argument('--max-voltage', type=float, default=None,
                        help='Max H-bridge power supply voltage [V] (optional override)')
    parser.add_argument('--dipole-strength', type=float, default=None,
                        help='Magnetorquer strength factor [Am^2/A] (optional override)')
    
    return parser.parse_args()


def main():
    args = get_args()
    fw_max_voltage, fw_dipole_strength, fw_source = read_firmware_calibration()

    eff_max_voltage = args.max_voltage if args.max_voltage is not None else HOST_DEFAULT_MAX_VOLTAGE
    eff_dipole_strength = args.dipole_strength if args.dipole_strength is not None else HOST_DEFAULT_DIPOLE_STRENGTH

    if eff_max_voltage <= 0.0:
        print("ERROR: effective max voltage must be > 0.", file=sys.stderr)
        sys.exit(2)
    if eff_dipole_strength <= 0.0:
        print("ERROR: effective dipole strength must be > 0.", file=sys.stderr)
        sys.exit(2)

    clamped_max_voltage = min(max(eff_max_voltage, FW_RUNTIME_MAX_VOLTAGE_MIN), FW_RUNTIME_MAX_VOLTAGE_MAX)
    clamped_dipole_strength = min(max(eff_dipole_strength, FW_RUNTIME_DIPOLE_MIN), FW_RUNTIME_DIPOLE_MAX)
    if not math.isclose(eff_max_voltage, clamped_max_voltage, rel_tol=0.0, abs_tol=1e-9):
        print(
            "WARNING: Effective max-voltage is outside firmware runtime range "
            f"[{FW_RUNTIME_MAX_VOLTAGE_MIN}, {FW_RUNTIME_MAX_VOLTAGE_MAX}]V. "
            f"Requested={eff_max_voltage:.6g}V -> Applied={clamped_max_voltage:.6g}V.",
            file=sys.stderr,
        )
    if not math.isclose(eff_dipole_strength, clamped_dipole_strength, rel_tol=0.0, abs_tol=1e-9):
        print(
            "WARNING: Effective dipole-strength is outside firmware runtime range "
            f"[{FW_RUNTIME_DIPOLE_MIN}, {FW_RUNTIME_DIPOLE_MAX}] Am^2/A. "
            f"Requested={eff_dipole_strength:.6g} -> Applied={clamped_dipole_strength:.6g}.",
            file=sys.stderr,
        )
    eff_max_voltage = clamped_max_voltage
    eff_dipole_strength = clamped_dipole_strength
    max_voltage_mV = max(0, min(65535, int(round(eff_max_voltage * 1000.0))))
    dipole_strength_milli = max(0, min(65535, int(round(eff_dipole_strength * 1000.0))))
    
    if not args.quiet:
        max_src = "CLI" if args.max_voltage is not None else "host default"
        dip_src = "CLI" if args.dipole_strength is not None else "host default"
        print("--- ADCS Simulation Host v3.0 (HITL) ---")
        print(f"Port: {args.port}")
        print(f"Gains: K_BDOT={args.kbdot:.0e}, K_P={args.kp}, K_I={args.ki}, K_D={args.kd}")
        print(f"Physics/FW override: MaxV={eff_max_voltage}V ({max_src}), Dipole={eff_dipole_strength}Am^2/A ({dip_src})")
        print(f"Override packet: max_voltage_mV={max_voltage_mV}, dipole_strength_milli={dipole_strength_milli}")
        print(f"Host defaults: MaxV={HOST_DEFAULT_MAX_VOLTAGE}V, Dipole={HOST_DEFAULT_DIPOLE_STRENGTH}Am^2/A")
        print(f"Firmware compile-time calibration: MaxV={fw_max_voltage}V, Dipole={fw_dipole_strength}Am^2/A")
        print(f"Firmware calib source: {fw_source}")
        print(f"Force mode: {args.force_mode}")
    
    # Initialize components
    sat = SatellitePhysics(max_voltage=eff_max_voltage, dipole_strength=eff_dipole_strength)
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

    if args.scenario == 'pointing':
        # Start with a 180 degree flip (q = [0, 1, 0, 0]) to test singularity logic
        sat.state[0:4] = np.array([0.0, 1.0, 0.0, 0.0])
    
    if not args.quiet:
        print(f"Scenario: {args.scenario}, Initial ω: {w_init}")
    
    # Connect to firmware
    try:
        comms = SerialInterface(args.port, BAUD_RATE)
    except Exception as e:
        print(f"ERROR: Cannot open port {args.port}: {e}")
        print("Make sure the Nucleo board is connected and the port is correct.")
        sys.exit(1)
    
    # Determine time step
    if args.dt:
        dt_val = args.dt
        if not args.quiet: print(f"Running with CUSTOM step size: {dt_val}s")
    elif args.realtime:
        dt_val = 1.0 
        if not args.quiet: print("Running in REALTIME mode (1s step, 90min orbit)")
    else:
        dt_val = 0.1
        if not args.quiet: print("Running in FAST mode (0.1s step, ~10 mins per orbit)")

    # Simulation state
    sim_time = 0.0
    step_count = 0
    torque_applied = np.array([0.0, 0.0, 0.0])
    sat_currents = np.array([0.0, 0.0, 0.0])
    firmware_voltages = np.array([0.0, 0.0, 0.0])
    adcs_mode_id = 0
    mode_counts = {0: 0, 1: 0, 2: 0, 3: 0}
    mode_transitions = 0
    last_mode_sample = None
    sat_axis_count = 0
    sat_axis_samples = 0
    proj_loss_sum = 0.0
    proj_loss_samples = 0
    int_clamp_count = 0
    int_clamp_samples = 0
    settle_time = None

    force_mode_map = {"auto": 0, "detumble": 1, "spin": 2, "pointing": 3}
    forced_mode_code = force_mode_map[args.force_mode]
    sat_voltage_ref = eff_max_voltage

    try:
        while True:
            # Check termination condition
            if args.duration > 0 and sim_time >= args.duration:
                q = sat.state[0:4]
                w = sat.state[4:7]
                qw, qx, qy, qz = q
                dot = max(-1.0, min(1.0, 1 - 2*(qx**2 + qy**2)))
                angle_err = math.degrees(math.acos(dot))
                total_mode_samples = sum(mode_counts.values())
                final_mode = int(adcs_mode_id)
                mode_dwell_ratio = (
                    mode_counts.get(final_mode, 0) / total_mode_samples
                    if total_mode_samples > 0 else 0.0
                )
                forced_mode_dwell = (
                    mode_counts.get(forced_mode_code, 0) / total_mode_samples
                    if forced_mode_code != 0 and total_mode_samples > 0 else -1.0
                )
                sat_voltage_ratio = (
                    sat_axis_count / sat_axis_samples if sat_axis_samples > 0 else 0.0
                )
                avg_proj_loss = (
                    proj_loss_sum / proj_loss_samples if proj_loss_samples > 0 else 0.0
                )
                int_clamp_ratio = (
                    int_clamp_count / int_clamp_samples if int_clamp_samples > 0 else 0.0
                )
                t_settle = settle_time if settle_time is not None else -1.0
                print(f"FINAL_STATE: W={np.linalg.norm(w):.6f} rad/s | Err={angle_err:.6f} deg")
                print(
                    "FINAL_METRICS: "
                    f"Mode={final_mode} "
                    f"Dwell={mode_dwell_ratio:.6f} "
                    f"ForcedDwell={forced_mode_dwell:.6f} "
                    f"Sat={sat_voltage_ratio:.6f} "
                    f"Transitions={mode_transitions} "
                    f"Tsettle={t_settle:.6f} "
                    f"ProjLoss={avg_proj_loss:.6f} "
                    f"IntClamp={int_clamp_ratio:.6f}"
                )
                break
            
            DT = dt_val
            t_start = time.perf_counter()
            
            # Get environment state
            B_inertial = sat.get_magnetic_field_inertial(sim_time)
            
            # Step physics forward
            sat.rk4_step(sim_time, DT, firmware_voltages, B_inertial)
            
            q = sat.state[0:4]
            w = sat.state[4:7]
            sat_currents = sat.state[7:10]
            
            # 5. Read Sensors (Post-Update to avoid skew)
            B_body = sat.get_b_field(q, B_inertial)
            
            # 6. Send to Firmware
            reset_request = 1 if args.reset_controller and step_count < 5 else 0
            debug_flags = (1 if args.open_loop else 0) | (forced_mode_code << 1) | (reset_request << 3)
            comms.send_packet(sat_currents.tolist(), w, B_body, q, 
                             args.kbdot, args.kp, args.ki, args.kd, DT,
                             debug_flags, max_voltage_mV, dipole_strength_milli)
            response = comms.read_packet()
            
            if response:
                firmware_voltages = np.array(response[0:3])
                packed_mode = int(response[3]) & 0xFF
                adcs_mode_id = packed_mode & 0x03
                int_clamp_flag = (packed_mode >> 2) & 0x01
                proj_loss_q5 = (packed_mode >> 3) & 0x1F
                proj_loss = proj_loss_q5 / 31.0
                if adcs_mode_id in mode_counts:
                    mode_counts[adcs_mode_id] += 1
                if last_mode_sample is not None and adcs_mode_id != last_mode_sample:
                    mode_transitions += 1
                last_mode_sample = adcs_mode_id
                proj_loss_sum += proj_loss
                proj_loss_samples += 1
                int_clamp_count += int_clamp_flag
                int_clamp_samples += 1

                sat_axis_count += int(np.sum(np.abs(firmware_voltages) >= 0.98 * sat_voltage_ref))
                sat_axis_samples += 3
            else:
                # No response - firmware may be busy, keep last values
                pass

            omega_norm = np.linalg.norm(w)
            qw, qx, qy, qz = q
            dot = max(-1.0, min(1.0, 1 - 2*(qx**2 + qy**2)))
            angle_err = math.degrees(math.acos(dot))
            if settle_time is None:
                if args.scenario == 'detumble' and omega_norm < 0.03:
                    settle_time = sim_time
                elif args.scenario == 'pointing' and omega_norm < 0.03 and angle_err < 10.0:
                    settle_time = sim_time
            
            # --- Telemetry (10Hz target) ---
            # Send every 10 steps if DT=0.01 (100Hz), or every 1 step if DT=0.1 (10Hz)
            # Calculate dynamic skip rate to keep telemetry ~10Hz
            telemetry_skip = max(1, int(0.1 / DT))
            
            if not args.quiet and step_count % telemetry_skip == 0:
                # Calculate torque for telemetry based on physics state current
                m_actual = sat_currents * sat.dipole_strength # Datasheet: 2.88 Am²/A (default)
                torque_applied = np.cross(m_actual, B_body)
            
            if not args.quiet and step_count % telemetry_skip == 0:
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
                if step_count % (telemetry_skip * 10) == 0:
                    print(f"t={sim_time:5.1f}s | Mode={m_str:10s} | W={np.linalg.norm(w):.3f} | Err={angle_err:5.1f}° | {stability_str} (P={work_rate:.2e}) | V={firmware_voltages}")
                    if args.debug:
                        print(f"  DEBUG: w={w} | B={B_body} | m={sat_currents * sat.dipole_strength}")
            
            sim_time += DT
            step_count += 1
            
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
