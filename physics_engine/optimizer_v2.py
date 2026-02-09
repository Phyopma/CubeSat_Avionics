#!/usr/bin/env python3
"""
ADCS Parameter Optimizer v2 - Runs detumble→pointing sequence for realistic tuning.
"""
import subprocess
import re
import time

def run_sim(kbdot, kp, kd, scenario, initial_omega, duration):
    """Run simulation and return final W and Error"""
    cmd = [
        "uv", "run", "simulation_host.py",
        "--scenario", scenario,
        "--kbdot", str(kbdot),
        "--kp", str(kp),
        "--kd", str(kd),
        "--initial_omega", *initial_omega.split(),
        "--duration", str(duration),
        "--port", "MOCK",
        "--quiet"
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, cwd="/Users/phyopyae/eecs159.tmp/physics_engine")
    
    for line in reversed(result.stdout.splitlines()):
        match = re.search(r"FINAL_STATE: W=([\d\.]+) rad/s \| Err=([\d\.]+) deg", line)
        if match:
            return float(match.group(1)), float(match.group(2))
    return 999.0, 999.0

def evaluate_params(kbdot, kp, kd, max_time=300):
    """
    Evaluate parameters with proper detumble→pointing sequence.
    Returns (success, final_W, final_Err, time_taken)
    """
    # Phase 1: Detumble from high omega
    w, _ = run_sim(kbdot, kp, kd, 
                   scenario="detumble", 
                   initial_omega="0.1 0.1 0.1",
                   duration=120)
    
    if w > 0.02:
        return False, w, 999.0, 120
    
    # Phase 2: Pointing from low omega (simulating post-detumble state)
    w_final, err_final = run_sim(kbdot, kp, kd,
                                  scenario="pointing",
                                  initial_omega="0.005 0.005 0.005",  # Very low starting W
                                  duration=180)
    
    return True, w_final, err_final, 300

print("=" * 50)
print("ADCS Parameter Optimizer v2 - Full Sequence Test")
print("=" * 50)

# Parameter grid - focusing on pointing stability
kbdot_values = [100000]  # Fixed kbdot that works well
kp_values = [0.0005, 0.001, 0.002, 0.005, 0.01]
kd_values = [0.01, 0.02, 0.05, 0.1]

results = []

for kbdot in kbdot_values:
    for kp in kp_values:
        for kd in kd_values:
            print(f"Testing: kbdot={kbdot}, kp={kp}, kd={kd} ... ", end="", flush=True)
            success, w, err, t = evaluate_params(kbdot, kp, kd)
            
            if success:
                print(f"✓ W={w:.4f}, Err={err:.1f}°")
                results.append((kbdot, kp, kd, w, err))
            else:
                print(f"✗ Detumble failed (W={w:.4f})")

if results:
    # Sort by: lowest error, then lowest W
    best = min(results, key=lambda x: (x[4], x[3]))
    
    print("\n" + "=" * 50)
    print("OPTIMAL PARAMETERS FOUND:")
    print("=" * 50)
    print(f"  kbdot = {best[0]}")
    print(f"  kp    = {best[1]}")
    print(f"  kd    = {best[2]}")
    print(f"  Final W = {best[3]:.5f} rad/s")
    print(f"  Final Error = {best[4]:.2f}°")
    print("=" * 50)
else:
    print("\nNo stable parameters found!")
