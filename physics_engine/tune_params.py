#!/usr/bin/env python3
"""
ADCS Parameter Tuner v2.0 - HITL Only
Tests both Detumble and Pointing modes on physical hardware.
Finds optimal K_BDOT for detumble and K_P/K_D for pointing.
"""
import subprocess
import re
import sys
import time

def run_sim(scenario, kbdot, kp, ki, kd, initial_omega, duration):
    """Run simulation and return final angular rate and error"""
    cmd = [
        "uv", "run", "simulation_host.py",
        "--scenario", scenario,
        "--kbdot", f"{kbdot:.1e}",
        "--kp", f"{kp:.3f}",
        "--ki", f"{ki:.5f}",
        "--kd", f"{kd:.3f}",
        "--initial_omega", *initial_omega.split(),
        "--duration", str(duration),
        "--quiet"
    ]
    # Note: No --port MOCK, uses default SERIAL_PORT in simulation_host.py
    # If a specific port is needed, it can be added to the cmd list.
    
    result = subprocess.run(cmd, capture_output=True, text=True, cwd="/Users/phyopyae/eecs159.tmp/physics_engine")
    
    for line in reversed(result.stdout.splitlines()):
        match = re.search(r"FINAL_STATE: W=([\d\.]+) rad/s \| Err=([\d\.]+) deg", line)
        if match:
            return float(match.group(1)), float(match.group(2))
    return 999.0, 999.0

def tune_detumble():
    """Tune K_BDOT for detumble mode"""
    print("\n" + "=" * 60)
    print("PHASE 1: DETUMBLE MODE (B-Dot) - Tuning K_BDOT")
    print("=" * 60)
    print("Goal: Minimize angular velocity from 0.86 rad/s tumble\n")
    
    INIT_OMEGA = "0.5 0.5 0.5" # 0.866 rad/s
    DURATION = 60 # 1 minute for faster sweep
    
    # Broad range 
    # Broad range - SKIPPING due to known best value from previous runs
    kbdot_values = [1e6] #, 8e6, 2e7, 5e7]
    results = []
    
    for kbdot in kbdot_values:
        print(f"Testing K_BDOT = {kbdot:.0e} ... ", end="", flush=True)
        w, _ = run_sim("detumble", kbdot, 0.05, 0.0, 0.20, INIT_OMEGA, DURATION)
        
        status = "✓" if w < 0.1 else "○" if w < 0.3 else "✗"
        print(f"{status} Final W = {w:.4f} rad/s")
        results.append((kbdot, w))
        time.sleep(1) # Brief pause between runs for UART stability
    
    results.sort(key=lambda x: x[1])
    best_kbdot = results[0][0]
    print(f"\n→ Best K_BDOT: {best_kbdot:.0e} (W = {results[0][1]:.4f} rad/s)")
    return best_kbdot

def tune_pointing(kbdot):
    """Tune K_P, K_I, and K_D for pointing mode"""
    print("\n" + "=" * 60)
    print("PHASE 2: POINTING MODE (PID Controller) - Tuning K_P/K_I/K_D")
    print("=" * 60)
    print("Goal: Minimize pointing error from stable initial state\n")
    
    INIT_OMEGA = "0.05 0.05 0.05"
    DURATION = 90 # 1.5 minutes
    
    # 3x3x3 parameter grid
    # 3x2x2 parameter grid (Reduced for speed)
    kp_values = [0.1, 0.15, 0.2]
    ki_values = [0.0001, 0.0002]
    kd_values = [0.1, 0.2]
    
    results = []
    
    for kp in kp_values:
        for ki in ki_values:
            for kd in kd_values:
                print(f"Testing K_P={kp:.3f}, K_I={ki:.5f}, K_D={kd:.3f} ... ", end="", flush=True)
                w, err = run_sim("pointing", kbdot, kp, ki, kd, INIT_OMEGA, DURATION)
                
                status = "✓" if err < 15 else "○" if err < 30 else "✗"
                print(f"{status} W={w:.4f}, Err={err:.1f}°")
                results.append((kp, ki, kd, w, err))
                time.sleep(1)
    
    # Sort by error primarily
    results.sort(key=lambda x: (x[4], x[3])) 
    
    print("\n" + "-" * 40)
    print("TOP 5 RESULTS:")
    for i, (kp, ki, kd, w, err) in enumerate(results[:5]):
        print(f"  {i+1}. K_P={kp:.3f}, K_I={ki:.5f}, K_D={kd:.3f} → Err={err:.1f}°, W={w:.4f}")
    
    best = results[0]
    return best[0], best[1], best[2], best[4]

def main():
    print("=" * 60)
    print("ADCS PARAMETER TUNER v2.0 (HITL)")
    print("=" * 60)
    print("Note: Each run takes 1-2 minutes. Total time depends on grid size.")
    
    # User can skip detumble phase if already tuned
    best_kbdot = tune_detumble()
    
    # Phase 2: Pointing
    best_kp, best_ki, best_kd, best_err = tune_pointing(best_kbdot)
    
    # Summary
    print("\n" + "=" * 60)
    print("OPTIMAL PARAMETERS FOUND (HITL)")
    print("=" * 60)
    print(f"  K_BDOT = {best_kbdot:.1e}   (Detumble gain)")
    print(f"  K_P    = {best_kp:.3f}       (Pointing proportional)")
    print(f"  K_I    = {best_ki:.5f}       (Pointing integral)")
    print(f"  K_D    = {best_kd:.3f}       (Pointing derivative)")
    print(f"\n  Final pointing error: {best_err:.1f}°")
    print("=" * 60)
    
    # Update config.h suggestion
    print("\nSuggested config.h updates:")
    print(f"  #define K_BDOT  {best_kbdot:.1e}f")
    print(f"  #define K_P     {best_kp:.3f}f")
    print(f"  #define K_I     {best_ki:.5f}f")
    print(f"  #define K_D     {best_kd:.3f}f")

if __name__ == "__main__":
    main()
