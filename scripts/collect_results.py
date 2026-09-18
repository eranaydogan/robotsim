"""Aggregates training runs into one report and a learning-curve figure.

Reads ``runs/<name>/summary.json`` and ``runs/<name>/evaluations.npz`` for the
given runs, evaluates the baseline policies with the same protocol, and writes
a Markdown table plus a plot of success rate against training steps.

Example:
    python scripts/collect_results.py --runs ppo-s0 ppo-s1 ppo-s2 \
        --output docs/results/phase3_training.md \
        --plot docs/results/phase3_learning_curve.png
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import numpy as np

import robotsim
from robotsim.baselines import RandomPolicy, RuleBasedPolicy
from robotsim.evaluate import EVAL_SEED_OFFSET, TABLE_HEADER, evaluate

ROOT = Path(__file__).resolve().parent.parent


def mean_std(values: list[float]) -> str:
    if len(values) == 1:
        return f"{values[0]:.1%}"
    return f"{statistics.mean(values):.1%} ± {statistics.stdev(values):.1%}"


def load_run(name: str) -> dict:
    summary_path = ROOT / "runs" / name / "summary.json"
    if not summary_path.exists():
        raise SystemExit(f"missing {summary_path}; train this run first")
    return json.loads(summary_path.read_text(encoding="utf-8"))


def plot_learning_curves(names: list[str], path: Path, baseline: float | None = None) -> None:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    figure = Figure(figsize=(7.0, 4.0), dpi=120)
    FigureCanvasAgg(figure)
    axes = figure.add_subplot(1, 1, 1)

    curves = []
    for name in names:
        npz_path = ROOT / "runs" / name / "evaluations.npz"
        if not npz_path.exists():
            continue
        data = np.load(npz_path)
        curves.append((data["timesteps"], data["successes"].mean(axis=1)))

    if not curves:
        return

    length = min(len(steps) for steps, _ in curves)
    steps = curves[0][0][:length]
    values = np.vstack([success[:length] for _, success in curves])

    axes.plot(steps, values.mean(axis=0), color="#1f77b4", label="PPO (mean of seeds)")
    if len(curves) > 1:
        axes.fill_between(
            steps, values.min(axis=0), values.max(axis=0), color="#1f77b4", alpha=0.2,
            label="PPO (min-max over seeds)",
        )
    if baseline is not None:
        axes.axhline(baseline, color="#ff7f0e", linestyle="--", linewidth=1.2,
                     label=f"rule-based baseline ({baseline:.0%})")
    axes.set_xlabel("environment steps")
    axes.set_ylabel("success rate (50 monitoring episodes)")
    axes.set_ylim(0.0, 1.0)
    axes.grid(alpha=0.3)
    axes.legend(loc="lower right", fontsize=9)
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", nargs="+", required=True, help="run names under runs/")
    parser.add_argument("--episodes", type=int, default=1000)
    parser.add_argument("--output", type=str, default="")
    parser.add_argument("--plot", type=str, default="")
    args = parser.parse_args()

    summaries = {name: load_run(name) for name in args.runs}
    config = robotsim.Config()

    lines = [
        "# robotsim Phase 3 training results",
        "",
        f"- Algorithm: PPO (Stable-Baselines3), {summaries[args.runs[0]]['args']['envs']} "
        f"parallel environments, {summaries[args.runs[0]]['args']['timesteps']:,} steps per run",
        f"- Seeds: {', '.join(str(s['args']['seed']) for s in summaries.values())}",
        f"- Evaluation: {args.episodes:,} episodes per policy on seeds "
        f"{EVAL_SEED_OFFSET:,} and above, deterministic actions",
        f"- Map: fixed warehouse layout, {len(config.obstacles)} obstacles, "
        f"episode limit {config.max_steps} steps",
        "",
        TABLE_HEADER,
    ]

    baseline_success = None
    for name, policy in [("random", RandomPolicy(seed=0)),
                         ("rule-based", RuleBasedPolicy(config))]:
        result = evaluate(policy, episodes=args.episodes)
        if name == "rule-based":
            baseline_success = result.success_rate
        lines.append(result.as_row(name))

    for name, summary in summaries.items():
        final = summary["final"]
        lines.append(
            f"| {name} | {final['success_rate']:.1%} | {final['collision_rate']:.1%} | "
            f"{final['timeout_rate']:.1%} | {final['mean_return']:.2f} | "
            f"{final['mean_length']:.1f} |"
        )

    successes = [s["final"]["success_rate"] for s in summaries.values()]
    collisions = [s["final"]["collision_rate"] for s in summaries.values()]
    timeouts = [s["final"]["timeout_rate"] for s in summaries.values()]
    lines.append(
        f"| **PPO ({len(summaries)} seeds)** | **{mean_std(successes)}** | "
        f"**{mean_std(collisions)}** | {mean_std(timeouts)} | | |"
    )

    seconds = [s["train_seconds"] for s in summaries.values()]
    rates = [s["steps_per_second"] for s in summaries.values()]
    lines += [
        "",
        "## Training",
        "",
        f"- Wall-clock time per run: {statistics.mean(seconds) / 60:.1f} min "
        f"({statistics.mean(rates):,.0f} environment steps per second)",
        f"- Hardware: {summaries[args.runs[0]]['platform']}",
        f"- Build: {summaries[args.runs[0]]['build_info']}",
        f"- Commit: {summaries[args.runs[0]]['commit']}",
        f"- Date: {time.strftime('%Y-%m-%d')}",
        "",
        "Reproduce with `python scripts/train.py --seed <n> --name ppo-s<n>` followed by "
        "`python scripts/collect_results.py --runs ppo-s0 ppo-s1 ppo-s2`.",
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

    if args.plot:
        plot_path = Path(args.plot)
        if not plot_path.is_absolute():
            plot_path = ROOT / plot_path
        plot_learning_curves(list(summaries), plot_path, baseline_success)
        print(f"Plot written to {plot_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())