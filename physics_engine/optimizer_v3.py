#!/usr/bin/env python3
"""
ADCS Parameter Optimizer v3 - Direct pointing test from near-zero omega.
Goal: Find parameters where error DECREASES over time and stabilizes.
"""
import subprocess
import re

def run_sim(kbdot, kp, kd, initial_omega, duration):
    """Run simulation and return final W and Error"""
    cmd = [
        "uv", "run", "simulation_host.py",
        "--scenario", "pointing",
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

print("=" * 60)
print("ADCS Parameter Optimizer v3 - Pointing Stability Test")
print("=" * 60)
print("Starting from near-zero omega to test pointing stability.\n")

# Very low initial omega (simulating post-detumble state)
INIT_OMEGA = "0.001 0.001 0.001"  # Below 0.02 threshold, will be in pointing mode

# Parameter grid
kp_values = [0.001, 0.005, 0.01, 0.02, 0.05, 0.1]
kd_values = [0.01, 0.02, 0.05, 0.1, 0.2]

results = []

for kp in kp_values:
    for kd in kd_values:
        print(f"Testing: kp={kp:.4f}, kd={kd:.3f} ... ", end="", flush=True)
        
        # Run 300s simulation
        w, err = run_sim(100000, kp, kd, INIT_OMEGA, 300)
        
        # Check if stable (error should be low and W should be low)
        status = "✓" if err < 10 else "○" if err < 30 else "✗"
        print(f"{status} W={w:.5f}, Err={err:.1f}°")
        
        results.append((kp, kd, w, err))

# Sort by lowest error
results.sort(key=lambda x: x[3])

print("\n" + "=" * 60)
print("TOP 5 RESULTS (sorted by lowest error):")
print("=" * 60)
for i, (kp, kd, w, err) in enumerate(results[:5]):
    print(f"  {i+1}. kp={kp:.4f}, kd={kd:.3f} → W={w:.5f}, Err={err:.1f}°")

best = results[0]
print("\n" + "=" * 60)
print("OPTIMAL PARAMETERS:")
print("=" * 60)
print(f"  kbdot = 100000")
print(f"  kp    = {best[0]}")  
print(f"  kd    = {best[1]}")
print(f"  Final W = {best[2]:.5f} rad/s")
print(f"  Final Error = {best[3]:.2f}°")
print("=" * 60)
