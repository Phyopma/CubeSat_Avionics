#!/usr/bin/env python3
"""
ADCS Parameter Tuner v4.0 - HITL Only
Flight-ready tuning with mode isolation, retry/backoff, and composite scoring.
"""

import math
import os
import random
import re
import statistics
import subprocess
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

ROOT = "/Users/phyopyae/eecs159/physics_engine"
SIM_SCRIPT = "simulation_host.py"
DEFAULT_PORT = "/dev/cu.usbmodem21303"
MAX_RETRIES = 3
RETRY_BASE_SEC = 0.5
BASELINE_KBDOT = 400000.0
BASELINE_KP = 0.0012
BASELINE_KI = 3e-05
BASELINE_KD = 0.17
BASELINE_C_DAMP = 0.0016405224
BASELINE_I_VIRTUAL = 0.0083109405


@dataclass
class RunResult:
    ok: bool
    scenario: str
    force_mode: str
    kbdot: float
    kp: float
    ki: float
    kd: float
    initial_omega: Tuple[float, float, float]
    duration: float
    w_final: float = 999.0
    err_deg: float = 999.0
    final_mode: int = -1
    mode_dwell: float = 0.0
    forced_mode_dwell: float = -1.0
    sat_ratio: float = 1.0
    proj_loss: float = 1.0
    int_clamp: float = 1.0
    transitions: int = 999
    t_settle: float = -1.0
    score: float = 1e9
    failure: str = ""


def is_port_busy(port: str) -> Tuple[bool, List[int]]:
    proc = subprocess.run(["lsof", "-t", port], capture_output=True, text=True)
    if proc.returncode != 0 or not proc.stdout.strip():
        return False, []

    pids = []
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line.isdigit():
            pids.append(int(line))
    return len(pids) > 0, pids


def parse_final(output: str) -> Tuple[Optional[float], Optional[float]]:
    state_re = re.compile(r"FINAL_STATE: W=([\d\.eE\+\-]+) rad/s \| Err=([\d\.eE\+\-]+) deg")
    for line in reversed(output.splitlines()):
        m = state_re.search(line)
        if m:
            return float(m.group(1)), float(m.group(2))
    return None, None


def parse_metrics(output: str) -> Dict[str, float]:
    key_map = {
        "Mode": ("final_mode", int),
        "Dwell": ("mode_dwell", float),
        "ForcedDwell": ("forced_mode_dwell", float),
        "Sat": ("sat_ratio", float),
        "Transitions": ("transitions", int),
        "Tsettle": ("t_settle", float),
        "ProjLoss": ("proj_loss", float),
        "IntClamp": ("int_clamp", float),
    }
    for line in reversed(output.splitlines()):
        if "FINAL_METRICS:" not in line:
            continue
        values: Dict[str, float] = {}
        for token in line.replace("FINAL_METRICS:", "").strip().split():
            if "=" not in token:
                continue
            key, raw = token.split("=", 1)
            if key not in key_map:
                continue
            out_key, caster = key_map[key]
            try:
                values[out_key] = caster(raw)
            except ValueError:
                continue
        if values:
            return values
    return {}


def run_sim(
    scenario: str,
    force_mode: str,
    kbdot: float,
    kp: float,
    ki: float,
    kd: float,
    initial_omega: Sequence[float],
    duration: float,
    port: str = DEFAULT_PORT,
) -> RunResult:
    assert len(initial_omega) == 3
    omega_tuple = (float(initial_omega[0]), float(initial_omega[1]), float(initial_omega[2]))

    for attempt in range(MAX_RETRIES):
        busy, pids = is_port_busy(port)
        if busy:
            return RunResult(
                ok=False,
                scenario=scenario,
                force_mode=force_mode,
                kbdot=kbdot,
                kp=kp,
                ki=ki,
                kd=kd,
                initial_omega=omega_tuple,
                duration=duration,
                failure=f"serial port busy: {port} held by {pids}",
            )

        cmd = [
            "uv", "run", SIM_SCRIPT,
            "--scenario", scenario,
            "--force-mode", force_mode,
            f"--kbdot={kbdot:.8g}",
            f"--kp={kp:.8g}",
            f"--ki={ki:.8g}",
            f"--kd={kd:.8g}",
            "--initial_omega",
            f"{omega_tuple[0]:.6f}", f"{omega_tuple[1]:.6f}", f"{omega_tuple[2]:.6f}",
            "--duration", f"{duration:.3f}",
            "--port", port,
            "--reset-controller",
            "--quiet",
        ]

        env = os.environ.copy()
        env.setdefault("UV_CACHE_DIR", os.path.join(ROOT, ".uv-cache"))
        proc = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT, env=env)
        stdout = proc.stdout or ""
        stderr = proc.stderr or ""

        w_final, err_deg = parse_final(stdout)
        metrics = parse_metrics(stdout)

        if proc.returncode == 0 and w_final is not None and err_deg is not None:
            return RunResult(
                ok=True,
                scenario=scenario,
                force_mode=force_mode,
                kbdot=kbdot,
                kp=kp,
                ki=ki,
                kd=kd,
                initial_omega=omega_tuple,
                duration=duration,
                w_final=w_final,
                err_deg=err_deg,
                final_mode=int(metrics.get("final_mode", -1)),
                mode_dwell=float(metrics.get("mode_dwell", 0.0)),
                forced_mode_dwell=float(metrics.get("forced_mode_dwell", -1.0)),
                sat_ratio=float(metrics.get("sat_ratio", 1.0)),
                proj_loss=float(metrics.get("proj_loss", 1.0)),
                int_clamp=float(metrics.get("int_clamp", 1.0)),
                transitions=int(metrics.get("transitions", 999)),
                t_settle=float(metrics.get("t_settle", -1.0)),
            )

        transient = (
            "SerialException" in stderr
            or "device reports readiness" in stderr
            or "Cannot open port" in stdout
        )
        if transient and attempt < MAX_RETRIES - 1:
            time.sleep(RETRY_BASE_SEC * (2 ** attempt))
            continue

        reason = f"run failed (code={proc.returncode})"
        if stderr.strip():
            reason += f": {stderr.strip().splitlines()[-1]}"
        elif stdout.strip():
            reason += f": {stdout.strip().splitlines()[-1]}"
        return RunResult(
            ok=False,
            scenario=scenario,
            force_mode=force_mode,
            kbdot=kbdot,
            kp=kp,
            ki=ki,
            kd=kd,
            initial_omega=omega_tuple,
            duration=duration,
            failure=reason,
        )

    return RunResult(
        ok=False,
        scenario=scenario,
        force_mode=force_mode,
        kbdot=kbdot,
        kp=kp,
        ki=ki,
        kd=kd,
        initial_omega=omega_tuple,
        duration=duration,
        failure="exhausted retries",
    )


def composite_score(r: RunResult, required_mode: Optional[int]) -> float:
    if not r.ok:
        return 1e9

    # Hard fail for forced-pointing runs with insufficient occupancy.
    if r.force_mode == "pointing" and r.forced_mode_dwell >= 0.0 and r.forced_mode_dwell < 0.95:
        return 1e8 + (0.95 - r.forced_mode_dwell) * 1e6

    # Keep occupancy checks for other forced modes.
    if required_mode is not None and r.force_mode in ("detumble", "spin"):
        if r.forced_mode_dwell >= 0.0 and r.forced_mode_dwell < 0.70:
            return 1e8 + (0.70 - r.forced_mode_dwell) * 1e6

    t_settle = r.t_settle if r.t_settle >= 0 else (1.5 * r.duration)
    mode_chatter = r.transitions / max(1.0, r.duration / 10.0)

    if r.scenario == "detumble":
        # For detumble scenarios, attitude error is not a control objective.
        score = (
            0.45 * r.w_final +
            0.25 * t_settle +
            0.20 * r.sat_ratio +
            0.10 * mode_chatter
        )
    else:
        score = (
            0.30 * r.err_deg +
            0.20 * t_settle +
            0.15 * r.sat_ratio +
            0.10 * mode_chatter +
            0.10 * r.w_final +
            0.10 * r.proj_loss +
            0.05 * r.int_clamp
        )

    if required_mode is not None and r.final_mode != required_mode:
        score += 500.0
    if r.force_mode == "auto" and r.scenario == "pointing" and r.final_mode != 3:
        score += 800.0

    return score


def format_result(r: RunResult) -> str:
    if not r.ok:
        return f"FAIL {r.failure}"
    return (
        f"W={r.w_final:.4f} Err={r.err_deg:.2f}deg Mode={r.final_mode} "
        f"FDwell={r.forced_mode_dwell:.3f} Sat={r.sat_ratio:.3f} "
        f"Proj={r.proj_loss:.3f} IntClamp={r.int_clamp:.3f} "
        f"Trans={r.transitions} Tset={r.t_settle:.2f}s Score={r.score:.3f}"
    )


def robust_evaluate(
    scenario_cases: Sequence[Tuple[str, Sequence[float], float]],
    force_mode: str,
    kbdot: float,
    kp: float,
    ki: float,
    kd: float,
    required_mode: Optional[int],
) -> RunResult:
    case_runs: List[RunResult] = []
    case_scores: List[float] = []

    for scenario, omega, duration in scenario_cases:
        run = run_sim(
            scenario=scenario,
            force_mode=force_mode,
            kbdot=kbdot,
            kp=kp,
            ki=ki,
            kd=kd,
            initial_omega=omega,
            duration=duration,
        )
        run.score = composite_score(run, required_mode=required_mode)
        case_runs.append(run)
        case_scores.append(run.score)

    rep = min(case_runs, key=lambda x: x.score)
    rep.score = statistics.median(case_scores) + 0.25 * max(case_scores)
    return rep


def logspace(low: float, high: float, n: int) -> List[float]:
    return [10 ** x for x in [math.log10(low) + i * (math.log10(high) - math.log10(low)) / (n - 1) for i in range(n)]]


def local_log_grid(center: float, span_frac: float, n: int) -> List[float]:
    low = center * (1.0 - span_frac)
    high = center * (1.0 + span_frac)
    if low <= 0:
        low = center * 0.1
    return logspace(low, high, n)


def latin_hypercube_2d(n: int, x_low: float, x_high: float, y_low: float, y_high: float, seed: int) -> List[Tuple[float, float]]:
    rng = random.Random(seed)
    bins = list(range(n))
    rng.shuffle(bins)

    samples = []
    for i in range(n):
        ux = (i + rng.random()) / n
        uy = (bins[i] + rng.random()) / n
        lx = math.log10(x_low) + ux * (math.log10(x_high) - math.log10(x_low))
        ly = math.log10(y_low) + uy * (math.log10(y_high) - math.log10(y_low))
        samples.append((10 ** lx, 10 ** ly))
    return samples


def run_detumble_search() -> Tuple[float, List[RunResult]]:
    cases = [
        ("detumble", (0.5, 0.5, 0.5), 90.0),
        ("detumble", (0.7, 0.2, 0.3), 90.0),
        ("detumble", (0.4, 0.6, 0.2), 90.0),
    ]

    coarse = logspace(2e4, 3e6, 15)
    coarse_results: List[RunResult] = []
    print("\n" + "=" * 80)
    print("DETUMBLE COARSE (forced detumble)")
    print("=" * 80)
    for idx, k in enumerate(coarse, start=1):
        r = robust_evaluate(
            scenario_cases=cases,
            force_mode="detumble",
            kbdot=k,
            kp=0.1,
            ki=0.0,
            kd=0.1,
            required_mode=1,
        )
        print(f"[{idx:03d}/{len(coarse):03d}] kbdot={k:.5g} {format_result(r)}")
        coarse_results.append(r)
    coarse_results.sort(key=lambda r: r.score)

    top3 = [r.kbdot for r in coarse_results[:3] if r.ok]
    refine_values: List[float] = []
    for c in top3:
        refine_values.extend(local_log_grid(c, span_frac=0.40, n=9))
    refine_values = sorted(set(round(v, 3) for v in refine_values))

    refine_results: List[RunResult] = []
    print("\n" + "=" * 80)
    print("DETUMBLE REFINE (forced detumble)")
    print("=" * 80)
    for idx, k in enumerate(refine_values, start=1):
        r = robust_evaluate(
            scenario_cases=cases,
            force_mode="detumble",
            kbdot=k,
            kp=0.1,
            ki=0.0,
            kd=0.1,
            required_mode=1,
        )
        print(f"[{idx:03d}/{len(refine_values):03d}] kbdot={k:.5g} {format_result(r)}")
        refine_results.append(r)
    refine_results.sort(key=lambda r: r.score)

    best = refine_results[0] if refine_results else coarse_results[0]
    return best.kbdot, (coarse_results + refine_results)


def run_baseline_matrix_snapshot() -> str:
    cases = [
        ("DT_A", "detumble", (0.5, 0.5, 0.5), 600.0),
        ("DT_B", "detumble", (0.7, 0.2, 0.3), 600.0),
        ("PT_A", "pointing", (0.05, 0.05, 0.05), 600.0),
        ("PT_B", "pointing", (0.08, 0.02, 0.03), 600.0),
    ]
    ts = time.strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(ROOT, "results")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"baseline_matrix_{ts}.txt")
    lines = [
        "BASELINE_MATRIX",
        f"K_BDOT={BASELINE_KBDOT}",
        f"K_P={BASELINE_KP}",
        f"K_I={BASELINE_KI}",
        f"K_D={BASELINE_KD}",
        "",
    ]

    print("\n" + "=" * 80)
    print("BASELINE MATRIX (AUTO MODE, 600s)")
    print("=" * 80)
    for name, scenario, omega, duration in cases:
        run = run_sim(
            scenario=scenario,
            force_mode="auto",
            kbdot=BASELINE_KBDOT,
            kp=BASELINE_KP,
            ki=BASELINE_KI,
            kd=BASELINE_KD,
            initial_omega=omega,
            duration=duration,
        )
        run.score = composite_score(run, required_mode=None)
        line = f"{name}: {format_result(run)}"
        print(line)
        lines.append(line)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Baseline snapshot saved: {out_path}")
    return out_path


def run_spin_search(kbdot: float) -> Tuple[Tuple[float, float], List[RunResult]]:
    cases = [
        ("detumble", (0.3, 0.1, 0.4), 90.0),
        ("detumble", (0.2, 0.5, 0.1), 90.0),
        ("detumble", (0.4, 0.3, 0.2), 90.0),
    ]

    coarse_pairs = latin_hypercube_2d(
        n=80,
        x_low=1e-3,
        x_high=3e-1,
        y_low=1e-4,
        y_high=5e-2,
        seed=42,
    )
    coarse_candidates = [(kbdot, c, i, "spin") for (c, i) in coarse_pairs]
    coarse_results = []

    print("\n" + "=" * 80)
    print("SPIN/KANE COARSE (forced spin)")
    print("=" * 80)
    for idx, (k, c_damp, i_virtual, mode) in enumerate(coarse_candidates, start=1):
        run = robust_evaluate(
            scenario_cases=cases,
            force_mode="spin",
            kbdot=k,
            kp=c_damp,
            ki=0.0,
            kd=i_virtual,
            required_mode=2,
        )
        print(f"[{idx:03d}/{len(coarse_candidates):03d}] c={c_damp:.5f} Id={i_virtual:.5f} {format_result(run)}")
        coarse_results.append(run)

    coarse_results.sort(key=lambda r: r.score)
    top10 = coarse_results[:10]

    refine_candidates = []
    for r in top10:
        for c in local_log_grid(r.kp, span_frac=0.40, n=5):
            for i in local_log_grid(r.kd, span_frac=0.40, n=5):
                refine_candidates.append((kbdot, c, i, "spin"))

    # Deduplicate to keep runtime bounded.
    dedup = {}
    for k, c, i, mode in refine_candidates:
        key = (round(c, 7), round(i, 7))
        dedup[key] = (k, c, i, mode)
    refine_candidates = list(dedup.values())

    refine_results = []
    print("\n" + "=" * 80)
    print("SPIN/KANE REFINE (forced spin)")
    print("=" * 80)
    for idx, (k, c_damp, i_virtual, mode) in enumerate(refine_candidates, start=1):
        run = robust_evaluate(
            scenario_cases=cases,
            force_mode="spin",
            kbdot=k,
            kp=c_damp,
            ki=0.0,
            kd=i_virtual,
            required_mode=2,
        )
        print(f"[{idx:03d}/{len(refine_candidates):03d}] c={c_damp:.5f} Id={i_virtual:.5f} {format_result(run)}")
        refine_results.append(run)

    combined = coarse_results + refine_results
    combined.sort(key=lambda r: r.score)
    best = combined[0]
    return (best.kp, best.kd), combined


def run_pointing_search(kbdot: float) -> Tuple[Tuple[float, float, float], List[RunResult]]:
    cases = [
        ("pointing", (0.05, 0.05, 0.05), 120.0),
        ("pointing", (0.08, 0.02, 0.03), 120.0),
        ("pointing", (0.03, 0.07, 0.04), 120.0),
    ]

    kp_vals = logspace(5e-4, 4e-3, 10)
    kd_vals = logspace(8e-2, 3.5e-1, 10)
    ki_vals = [0.0, 2e-6, 1e-5, 3e-5, 6e-5, 1e-4, 1.5e-4]

    coarse_results = []
    total = len(kp_vals) * len(kd_vals) * len(ki_vals)

    print("\n" + "=" * 80)
    print("POINTING COARSE (forced pointing)")
    print("=" * 80)
    idx = 0
    for kp in kp_vals:
        for kd in kd_vals:
            for ki in ki_vals:
                idx += 1
                run = robust_evaluate(
                    scenario_cases=cases,
                    force_mode="pointing",
                    kbdot=kbdot,
                    kp=kp,
                    ki=ki,
                    kd=kd,
                    required_mode=3,
                )
                print(f"[{idx:04d}/{total:04d}] kp={kp:.6f} ki={ki:.6f} kd={kd:.6f} {format_result(run)}")
                coarse_results.append(run)

    coarse_results.sort(key=lambda r: r.score)
    top5 = coarse_results[:5]

    refine_candidates = []
    for r in top5:
        for kp in [r.kp * s for s in (0.8, 1.0, 1.2) if r.kp * s > 0]:
            for ki in [r.ki * s for s in (0.8, 1.0, 1.2)] + [0.0]:
                if ki < 0:
                    continue
                for kd in [r.kd * s for s in (0.8, 1.0, 1.2) if r.kd * s > 0]:
                    refine_candidates.append((kp, ki, kd))

    dedup = {}
    for kp, ki, kd in refine_candidates:
        dedup[(round(kp, 9), round(ki, 9), round(kd, 9))] = (kp, ki, kd)
    refine_candidates = list(dedup.values())

    refine_results = []
    print("\n" + "=" * 80)
    print("POINTING REFINE (forced pointing)")
    print("=" * 80)
    for idx, (kp, ki, kd) in enumerate(refine_candidates, start=1):
        run = robust_evaluate(
            scenario_cases=cases,
            force_mode="pointing",
            kbdot=kbdot,
            kp=kp,
            ki=ki,
            kd=kd,
            required_mode=3,
        )
        print(f"[{idx:03d}/{len(refine_candidates):03d}] kp={kp:.6f} ki={ki:.6f} kd={kd:.6f} {format_result(run)}")
        refine_results.append(run)

    combined = coarse_results + refine_results
    combined.sort(key=lambda r: r.score)
    best = combined[0]
    return (best.kp, best.ki, best.kd), combined


def integrated_validation(
    kbdot: float,
    spin_params: Tuple[float, float],
    pid_params: Tuple[float, float, float],
) -> List[RunResult]:
    c_damp, i_virtual = spin_params
    kp, ki, kd = pid_params

    # Because packet size is fixed, spin params are pushed through a short forced-spin
    # pre-arm run, then auto-mode validation is executed with pointing gains.
    print("\n" + "=" * 80)
    print("INTEGRATED VALIDATION")
    print("=" * 80)

    prearm = run_sim(
        scenario="detumble",
        force_mode="spin",
        kbdot=kbdot,
        kp=c_damp,
        ki=0.0,
        kd=i_virtual,
        initial_omega=(0.3, 0.1, 0.4),
        duration=5.0,
    )
    prearm.score = composite_score(prearm, required_mode=2)
    print(f"[PREARM] c={c_damp:.6f} Id={i_virtual:.6f} {format_result(prearm)}")

    test_cases = [
        ("detumble_hi", "detumble", (0.5, 0.5, 0.5), 180.0),
        ("pointing_flip", "pointing", (0.05, 0.05, 0.05), 180.0),
        ("mixed_spin", "detumble", (0.4, 0.2, 0.3), 180.0),
    ]

    results = [prearm]
    for name, scenario, omega, duration in test_cases:
        run = run_sim(
            scenario=scenario,
            force_mode="auto",
            kbdot=kbdot,
            kp=kp,
            ki=ki,
            kd=kd,
            initial_omega=omega,
            duration=duration,
        )
        run.score = composite_score(run, required_mode=None)
        print(f"[{name}] {format_result(run)}")
        results.append(run)

    return results


def summarize_top(title: str, results: List[RunResult], n: int = 10) -> None:
    print("\n" + "-" * 80)
    print(title)
    print("-" * 80)
    for i, r in enumerate(results[:n], start=1):
        if r.force_mode == "spin":
            param = f"c={r.kp:.6f} Id={r.kd:.6f}"
        else:
            param = f"KBDOT={r.kbdot:.4g} KP={r.kp:.6f} KI={r.ki:.6f} KD={r.kd:.6f}"
        print(f"{i:2d}. {param} -> {format_result(r)}")


def main() -> None:
    print("=" * 80)
    print("ADCS PARAMETER TUNER v4.0 (HITL)")
    print("Objective: flight-ready balance")
    print("=" * 80)

    # Stage A: locked baseline and reproducibility snapshot
    run_baseline_matrix_snapshot()
    best_kbdot = BASELINE_KBDOT
    best_spin = (BASELINE_C_DAMP, BASELINE_I_VIRTUAL)

    # Stage B: Pointing PID (forced pointing), with K_BDOT held fixed
    best_pid, pointing_results = run_pointing_search(best_kbdot)
    summarize_top("Pointing top candidates", pointing_results)

    # Stage C: Integrated validation in auto mode
    integrated = integrated_validation(best_kbdot, best_spin, best_pid)

    print("\n" + "=" * 80)
    print("SELECTED PARAMETERS")
    print("=" * 80)
    print(f"K_BDOT      = {best_kbdot:.8g}")
    print(f"C_DAMP      = {best_spin[0]:.8g}")
    print(f"I_VIRTUAL   = {best_spin[1]:.8g}")
    print(f"K_P         = {best_pid[0]:.8g}")
    print(f"K_I         = {best_pid[1]:.8g}")
    print(f"K_D         = {best_pid[2]:.8g}")

    print("\nSuggested config.h updates:")
    print(f"#define K_BDOT      {best_kbdot:.8g}f")
    print(f"#define C_DAMP      {best_spin[0]:.8g}f")
    print(f"#define I_VIRTUAL   {best_spin[1]:.8g}f")
    print(f"#define K_P         {best_pid[0]:.8g}f")
    print(f"#define K_I         {best_pid[1]:.8g}f")
    print(f"#define K_D         {best_pid[2]:.8g}f")

    ok_integrated = [r for r in integrated if r.ok]
    if ok_integrated:
        agg = statistics.mean(r.score for r in ok_integrated)
        print(f"\nIntegrated mean score: {agg:.4f}")


if __name__ == "__main__":
    main()
