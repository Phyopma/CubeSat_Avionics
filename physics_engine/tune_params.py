#!/usr/bin/env python3
"""
ADCS Parameter Tuner v3.0 - HITL Only (3-Phase Tuning)
Comprehensive tuning with coarse -> fine -> ultra-fine sweeps.
Hardware parameters: 2.88 Am²/A dipole factor, 3.3V supply
"""
import subprocess
import re
import sys
import time
import numpy as np

def run_sim(scenario, kbdot, kp, ki, kd, initial_omega, duration):
    """Run simulation and return final angular rate and error"""
    cmd = [
        "uv", "run", "simulation_host.py",
        "--scenario", scenario,
        f"--kbdot={kbdot:.0f}",
        f"--kp={kp:.4f}",
        f"--ki={ki:.6f}",
        f"--kd={kd:.4f}",
        "--initial_omega", *initial_omega.split(),
        "--duration", str(duration),
        "--quiet"
    ]

    result = subprocess.run(cmd, capture_output=True, text=True, cwd="/Users/phyopyae/eecs159.tmp/physics_engine")

    for line in reversed(result.stdout.splitlines()):
        match = re.search(r"FINAL_STATE: W=([\d\.]+) rad/s \| Err=([\d\.]+) deg", line)
        if match:
            return float(match.group(1)), float(match.group(2))
    return 999.0, 999.0

def tune_detumble_coarse():
    """Phase 1a: Coarse K_BDOT sweep"""
    print("\n" + "=" * 70)
    print("PHASE 1a: DETUMBLE - COARSE SWEEP")
    print("=" * 70)

    INIT_OMEGA = "0.5 0.5 0.5"  # 0.866 rad/s
    DURATION = 60

    # Positive values required for damping: M = k * (w x B)
    kbdot_values = [200000, 400000, 600000, 800000, 1000000, 1500000, 2000000]
    results = []

    for kbdot in kbdot_values:
        print(f"Testing K_BDOT = {kbdot:.0e} ... ", end="", flush=True)
        w, _ = run_sim("detumble", kbdot, 0.30, 0.0005, 0.30, INIT_OMEGA, DURATION)

        status = "✓" if w < 0.05 else "○" if w < 0.1 else "✗"
        print(f"{status} Final W = {w:.4f} rad/s")
        results.append((kbdot, w))
        time.sleep(0.5)

    results.sort(key=lambda x: x[1])
    best_kbdot = results[0][0]
    print(f"\n→ Best coarse K_BDOT: {best_kbdot:.0e} (W = {results[0][1]:.4f} rad/s)")
    return best_kbdot

def tune_detumble_fine(best_coarse_kbdot):
    """Phase 1b: Fine K_BDOT sweep around best coarse"""
    print("\n" + "=" * 70)
    print("PHASE 1b: DETUMBLE - FINE SWEEP")
    print("=" * 70)

    INIT_OMEGA = "0.5 0.5 0.5"
    DURATION = 90

    # Fine grid: step 50000 around best coarse
    center = best_coarse_kbdot
    kbdot_values = [center + delta for delta in [-100000, -50000, 0, 50000, 100000]]
    kbdot_values = [v for v in kbdot_values if v > 0] # Must stay positive
    results = []

    for kbdot in kbdot_values:
        print(f"Testing K_BDOT = {kbdot:.0e} ... ", end="", flush=True)
        w, _ = run_sim("detumble", kbdot, 0.30, 0.0005, 0.30, INIT_OMEGA, DURATION)

        status = "✓" if w < 0.03 else "○" if w < 0.05 else "✗"
        print(f"{status} Final W = {w:.4f} rad/s")
        results.append((kbdot, w))
        time.sleep(0.5)

    results.sort(key=lambda x: x[1])
    best_kbdot = results[0][0]
    print(f"\n→ Best fine K_BDOT: {best_kbdot:.0e} (W = {results[0][1]:.4f} rad/s)")
    return best_kbdot

def tune_pointing_coarse(kbdot):
    """Phase 2a: Coarse PID sweep"""
    print("\n" + "=" * 70)
    print("PHASE 2a: POINTING - COARSE SWEEP")
    print("=" * 70)

    INIT_OMEGA = "0.05 0.05 0.05"
    DURATION = 90

    # Coarse grid: ~3x higher due to reduced torque authority
    kp_values = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80]
    ki_values = [0.0001, 0.0002, 0.0005, 0.001, 0.002]
    kd_values = [0.10, 0.20, 0.30, 0.40, 0.50]

    results = []
    total = len(kp_values) * len(ki_values) * len(kd_values)
    count = 0

    for kp in kp_values:
        for ki in ki_values:
            for kd in kd_values:
                count += 1
                print(f"[{count}/{total}] K_P={kp:.2f}, K_I={ki:.5f}, K_D={kd:.2f} ... ", end="", flush=True)
                w, err = run_sim("pointing", kbdot, kp, ki, kd, INIT_OMEGA, DURATION)

                status = "✓" if err < 10 else "○" if err < 20 else "✗"
                print(f"{status} Err={err:.1f}°, W={w:.4f}")
                results.append((kp, ki, kd, w, err))
                time.sleep(0.5)

    results.sort(key=lambda x: (x[4], x[3]))

    print("\n" + "-" * 50)
    print("TOP 10 COARSE RESULTS:")
    for i, (kp, ki, kd, w, err) in enumerate(results[:10]):
        print(f"  {i+1}. K_P={kp:.2f}, K_I={ki:.5f}, K_D={kd:.2f} → Err={err:.1f}°, W={w:.4f}")

    best = results[0]
    return best[0], best[1], best[2], best[4]

def tune_pointing_fine(kbdot, best_kp, best_ki, best_kd):
    """Phase 2b: Fine PID sweep around best coarse"""
    print("\n" + "=" * 70)
    print("PHASE 2b: POINTING - FINE SWEEP")
    print("=" * 70)

    INIT_OMEGA = "0.05 0.05 0.05"
    DURATION = 120

    # Fine grid: step 0.02 for kp/kd, 0.0001 for ki
    kp_values = [best_kp + d for d in [-0.04, -0.02, 0, 0.02, 0.04]]
    ki_values = [best_ki + d for d in [-0.0002, -0.0001, 0, 0.0001, 0.0002]]
    kd_values = [best_kd + d for d in [-0.04, -0.02, 0, 0.02, 0.04]]

    # Filter out negative values
    kp_values = [v for v in kp_values if v > 0]
    ki_values = [v for v in ki_values if v > 0]
    kd_values = [v for v in kd_values if v > 0]

    results = []
    total = len(kp_values) * len(ki_values) * len(kd_values)
    count = 0

    for kp in kp_values:
        for ki in ki_values:
            for kd in kd_values:
                count += 1
                print(f"[{count}/{total}] K_P={kp:.3f}, K_I={ki:.6f}, K_D={kd:.3f} ... ", end="", flush=True)
                w, err = run_sim("pointing", kbdot, kp, ki, kd, INIT_OMEGA, DURATION)

                status = "✓" if err < 8 else "○" if err < 12 else "✗"
                print(f"{status} Err={err:.1f}°, W={w:.4f}")
                results.append((kp, ki, kd, w, err))
                time.sleep(0.5)

    results.sort(key=lambda x: (x[4], x[3]))

    print("\n" + "-" * 50)
    print("TOP 10 FINE RESULTS:")
    for i, (kp, ki, kd, w, err) in enumerate(results[:10]):
        print(f"  {i+1}. K_P={kp:.3f}, K_I={ki:.6f}, K_D={kd:.3f} → Err={err:.1f}°, W={w:.4f}")

    best = results[0]
    return best[0], best[1], best[2], best[4]

def tune_pointing_ultrafine(kbdot, best_kp, best_ki, best_kd):
    """Phase 2c: Ultra-fine PID sweep for perfect params"""
    print("\n" + "=" * 70)
    print("PHASE 2c: POINTING - ULTRA-FINE SWEEP")
    print("=" * 70)

    INIT_OMEGA = "0.05 0.05 0.05"
    DURATION = 150

    # Ultra-fine grid: step 0.005 for kp/kd, 0.00002 for ki
    kp_values = [best_kp + d for d in [-0.010, -0.005, 0, 0.005, 0.010]]
    ki_values = [best_ki + d for d in [-0.00004, -0.00002, 0, 0.00002, 0.00004]]
    kd_values = [best_kd + d for d in [-0.010, -0.005, 0, 0.005, 0.010]]

    # Filter out negative values
    kp_values = [v for v in kp_values if v > 0]
    ki_values = [v for v in ki_values if v > 0]
    kd_values = [v for v in kd_values if v > 0]

    results = []
    total = len(kp_values) * len(ki_values) * len(kd_values)
    count = 0

    for kp in kp_values:
        for ki in ki_values:
            for kd in kd_values:
                count += 1
                print(f"[{count}/{total}] K_P={kp:.4f}, K_I={ki:.6f}, K_D={kd:.4f} ... ", end="", flush=True)
                w, err = run_sim("pointing", kbdot, kp, ki, kd, INIT_OMEGA, DURATION)

                status = "✓" if err < 5 else "○" if err < 8 else "✗"
                print(f"{status} Err={err:.2f}°, W={w:.4f}")
                results.append((kp, ki, kd, w, err))
                time.sleep(0.5)

    results.sort(key=lambda x: (x[4], x[3]))

    print("\n" + "-" * 50)
    print("TOP 10 ULTRA-FINE RESULTS:")
    for i, (kp, ki, kd, w, err) in enumerate(results[:10]):
        print(f"  {i+1}. K_P={kp:.4f}, K_I={ki:.6f}, K_D={kd:.4f} → Err={err:.2f}°, W={w:.4f}")

    best = results[0]
    return best[0], best[1], best[2], best[4]

def main():
    print("=" * 70)
    print("ADCS PARAMETER TUNER v3.0 (HITL) - 3-Phase Tuning")
    print("=" * 70)
    print("Hardware: 2.88 Am²/A dipole factor, 3.3V supply")
    print("Note: Full tuning takes ~2-3 hours. Coarse sweep finds region,")
    print("      fine sweep refines, ultra-fine finds perfect params.")
    print()

    # Phase 1: Detumble tuning
    best_kbdot = tune_detumble_coarse()
    best_kbdot = tune_detumble_fine(best_kbdot)

    # Phase 2a: Coarse pointing sweep
    best_kp, best_ki, best_kd, _ = tune_pointing_coarse(best_kbdot)

    # Phase 2b: Fine pointing sweep
    best_kp, best_ki, best_kd, _ = tune_pointing_fine(best_kbdot, best_kp, best_ki, best_kd)

    # Phase 2c: Ultra-fine pointing sweep
    best_kp, best_ki, best_kd, best_err = tune_pointing_ultrafine(best_kbdot, best_kp, best_ki, best_kd)

    # Summary
    print("\n" + "=" * 70)
    print("OPTIMAL PARAMETERS FOUND (PERFECT TUNING)")
    print("=" * 70)
    print(f"  K_BDOT = {best_kbdot:.1e}   (Detumble gain)")
    print(f"  K_P    = {best_kp:.4f}      (Pointing proportional)")
    print(f"  K_I    = {best_ki:.6f}      (Pointing integral)")
    print(f"  K_D    = {best_kd:.4f}      (Pointing derivative)")
    print(f"\n  Final pointing error: {best_err:.2f}°")
    print("=" * 70)

    # Update config.h suggestion
    print("\nSuggested config.h updates:")
    print(f"  #define K_BDOT  {best_kbdot:.1e}f")
    print(f"  #define K_P     {best_kp:.4f}f")
    print(f"  #define K_I     {best_ki:.6f}f")
    print(f"  #define K_D     {best_kd:.4f}f")

if __name__ == "__main__":
    main()
