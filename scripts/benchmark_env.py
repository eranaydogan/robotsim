"""Benchmarks the Python layers on top of the C++ core.

Measures three layers on the same workload:

* ``World.step``                      bindings only, no observation
* ``World.step`` + ``observation``    bindings plus one NumPy array per step
* ``RobotNavEnv.step``                the full Gymnasium interface

Run ``python scripts/benchmark_env.py --output docs/results/phase2_speed.md``.
The C++ core benchmark (``scripts/run_cpp_benchmark.ps1``) provides the
native reference figure to compare against.
"""

from __future__ import annotations

import argparse
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable

import gymnasium as gym
import numpy as np

import robotsim

ROOT = Path(__file__).resolve().parent.parent


def measure(steps: int, reps: int, body: Callable[[int], None]) -> tuple[float, float, float]:
    """Runs one warm-up and `reps` timed repetitions; returns median, min, max rates."""
    body(steps)
    rates = []
    for _ in range(reps):
        start = time.perf_counter()
        body(steps)
        rates.append(steps / (time.perf_counter() - start))
    return statistics.median(rates), min(rates), max(rates)


def bench_world_step(steps: int) -> None:
    world = robotsim.World()
    world.reset(1)
    rng = np.random.default_rng(5)
    actions = rng.uniform(-1.0, 1.0, size=(steps, 2))
    for action in actions:
        world.step(float((action[0] + 1.0) * 0.5), float(action[1] * 1.5))
        if world.done:
            world.reset()


def bench_world_step_observation(steps: int) -> None:
    world = robotsim.World()
    world.reset(1)
    rng = np.random.default_rng(5)
    actions = rng.uniform(-1.0, 1.0, size=(steps, 2))
    for action in actions:
        world.step(float((action[0] + 1.0) * 0.5), float(action[1] * 1.5))
        world.observation()
        if world.done:
            world.reset()


def bench_env_step(steps: int) -> None:
    env = robotsim.RobotNavEnv()
    env.reset(seed=1)
    rng = np.random.default_rng(5)
    actions = rng.uniform(-1.0, 1.0, size=(steps, 2))
    for action in actions:
        _, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            env.reset()


def cpu_name() -> str:
    try:
        import winreg

        key = winreg.OpenKey(
            winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0"
        )
        with key:
            return str(winreg.QueryValueEx(key, "ProcessorNameString")[0]).strip()
    except Exception:
        return platform.processor() or "unknown"


def git_commit() -> str:
    try:
        commit = subprocess.run(
            ["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "-C", str(ROOT), "status", "--porcelain"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        return commit + (" (with uncommitted changes)" if dirty else "")
    except Exception:
        return "unknown"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=50_000, help="steps per repetition")
    parser.add_argument("--reps", type=int, default=5, help="timed repetitions")
    parser.add_argument("--no-pin", action="store_true", help="do not pin to a performance core")
    parser.add_argument("--output", type=str, default="", help="write a Markdown report here")
    args = parser.parse_args()

    pinned = "no"
    if not args.no_pin:
        where = robotsim.pin_to_performance_core()
        pinned = where or "requested but not available"

    benchmarks = [
        ("World.step", bench_world_step),
        ("World.step + observation", bench_world_step_observation),
        ("RobotNavEnv.step", bench_env_step),
    ]

    lines = [
        "# robotsim Python interface benchmark",
        "",
        f"- Build: {robotsim.build_info()}",
        f"- Python {platform.python_version()}, NumPy {np.__version__}, "
        f"Gymnasium {gym.__version__}",
        f"- Steps per repetition: {args.steps:,}",
        f"- Repetitions: {args.reps} timed runs after one warm-up, single thread",
        f"- Thread pinned: {pinned}",
        "",
        "| Layer | Median /s | Min /s | Max /s |",
        "|---|---|---|---|",
    ]
    for name, body in benchmarks:
        median, low, high = measure(args.steps, args.reps, body)
        lines.append(f"| {name} | {median:,.0f} | {low:,.0f} | {high:,.0f} |")

    lines += [
        "",
        "## Machine",
        "",
        f"- CPU: {cpu_name()}",
        f"- OS: {platform.platform()}",
        f"- Commit: {git_commit()}",
        f"- Date: {time.strftime('%Y-%m-%d')}",
        "",
        "Reproduce with `python scripts/benchmark_env.py`.",
    ]

    report = "\n".join(lines) + "\n"
    print(report)

    if args.output:
        out = Path(args.output)
        if not out.is_absolute():
            out = ROOT / out
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(report, encoding="utf-8", newline="\n")
        print(f"Report written to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())