import argparse
import itertools
import math
from dataclasses import dataclass


@dataclass
class PlantConfig:
    resistance: float = 25.0
    inductance: float = 0.5
    dt: float = 0.001
    sim_time: float = 2.0
    rail_voltage: float = 5.0
    target_current: float = 0.04


@dataclass
class Score:
    kp: float
    ki: float
    cost: float
    iae: float
    rmse: float
    overshoot_pct: float
    settling_time_s: float
    control_rms_v: float


def linspace(start: float, stop: float, steps: int) -> list[float]:
    if steps <= 1:
        return [start]
    step = (stop - start) / (steps - 1)
    return [start + i * step for i in range(steps)]


def simulate_step(config: PlantConfig, kp: float, ki: float) -> Score:
    target = config.target_current
    current = 0.0
    integral_error = 0.0

    sq_error = 0.0
    abs_error = 0.0
    peak_current = -float("inf")
    control_sq = 0.0

    settle_band = max(0.02 * abs(target), 1e-6)
    last_out_of_band_time = 0.0

    steps = int(config.sim_time / config.dt)
    for i in range(steps):
        t = i * config.dt
        error = target - current
        integral_error += error * config.dt

        control_voltage = (kp * error) + (ki * integral_error)
        control_voltage = max(min(control_voltage, config.rail_voltage), -config.rail_voltage)

        # RL plant: V = R*I + L*(dI/dt)
        di_dt = (control_voltage - (config.resistance * current)) / config.inductance
        current += di_dt * config.dt

        sq_error += error * error
        abs_error += abs(error) * config.dt
        control_sq += control_voltage * control_voltage
        peak_current = max(peak_current, current)

        if abs(target - current) > settle_band:
            last_out_of_band_time = t

    rmse = math.sqrt(sq_error / max(steps, 1))
    control_rms = math.sqrt(control_sq / max(steps, 1))

    if target != 0:
        overshoot = max(0.0, (peak_current - target) / abs(target) * 100.0)
    else:
        overshoot = 0.0

    settling_time = min(config.sim_time, last_out_of_band_time + config.dt)

    # Weighted objective tuned for fast/accurate/stable current control
    cost = (
        (200.0 * abs_error)
        + (120.0 * rmse)
        + (0.06 * overshoot)
        + (3.0 * settling_time)
        + (0.03 * control_rms)
    )

    return Score(
        kp=kp,
        ki=ki,
        cost=cost,
        iae=abs_error,
        rmse=rmse,
        overshoot_pct=overshoot,
        settling_time_s=settling_time,
        control_rms_v=control_rms,
    )


def run_sweep(config: PlantConfig, omega_min: float, omega_max: float, omega_steps: int, alpha_min: float, alpha_max: float, alpha_steps: int, top_n: int) -> list[Score]:
    # Physics-guided seed formulas for RL + PI:
    #   kp ~ L * wc
    #   ki ~ alpha * R * wc  where alpha near 1 cancels plant pole (-R/L)
    omega_values = linspace(omega_min, omega_max, omega_steps)
    alpha_values = linspace(alpha_min, alpha_max, alpha_steps)

    results: list[Score] = []
    for omega, alpha in itertools.product(omega_values, alpha_values):
        kp = config.inductance * omega
        ki = alpha * config.resistance * omega
        results.append(simulate_step(config, kp=kp, ki=ki))

    results.sort(key=lambda r: r.cost)
    return results[:top_n]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PI parameter sweep for current-control RL plant")
    parser.add_argument("--resistance", type=float, default=25.0)
    parser.add_argument("--inductance", type=float, default=0.5)
    parser.add_argument("--dt", type=float, default=0.001, help="Simulation timestep (s)")
    parser.add_argument("--sim-time", type=float, default=2.0)
    parser.add_argument("--target", type=float, default=0.04, help="Step target current (A)")

    parser.add_argument("--omega-min", type=float, default=8.0, help="Min crossover-like omega (rad/s)")
    parser.add_argument("--omega-max", type=float, default=90.0, help="Max crossover-like omega (rad/s)")
    parser.add_argument("--omega-steps", type=int, default=70)

    parser.add_argument("--alpha-min", type=float, default=0.5, help="Min ki scaling around pole-cancel baseline")
    parser.add_argument("--alpha-max", type=float, default=1.8)
    parser.add_argument("--alpha-steps", type=int, default=65)

    parser.add_argument("--top", type=int, default=12, help="Number of best candidates to print")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = PlantConfig(
        resistance=args.resistance,
        inductance=args.inductance,
        dt=args.dt,
        sim_time=args.sim_time,
        target_current=args.target,
    )

    best = run_sweep(
        config,
        omega_min=args.omega_min,
        omega_max=args.omega_max,
        omega_steps=args.omega_steps,
        alpha_min=args.alpha_min,
        alpha_max=args.alpha_max,
        alpha_steps=args.alpha_steps,
        top_n=args.top,
    )

    print("Top PI candidates (sorted by objective cost):")
    print("rank | Kp      | Ki       | cost    | IAE      | RMSE     | overshoot% | settle_s | Vrms")
    print("-----+---------+----------+---------+----------+----------+------------+----------+------")
    for idx, row in enumerate(best, start=1):
        print(
            f"{idx:>4} | {row.kp:>7.4f} | {row.ki:>8.4f} | {row.cost:>7.3f} | "
            f"{row.iae:>8.6f} | {row.rmse:>8.6f} | {row.overshoot_pct:>10.3f} | "
            f"{row.settling_time_s:>8.4f} | {row.control_rms_v:>4.2f}"
        )

    if best:
        winner = best[0]
        print("\nRecommended parameters:")
        print(f"Kp={winner.kp:.6f}, Ki={winner.ki:.6f}")


if __name__ == "__main__":
    main()
