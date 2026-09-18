"""Evaluates the reference policies and writes a Markdown report.

Run ``python scripts/evaluate_baselines.py --output docs/results/phase3_baselines.md``.
The same protocol (``robotsim.evaluate``) is used later for trained agents, so
the numbers are directly comparable.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import robotsim
from robotsim.baselines import RandomPolicy, RuleBasedPolicy
from robotsim.evaluate import EVAL_SEED_OFFSET, TABLE_HEADER, evaluate

ROOT = Path(__file__).resolve().parent.parent


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
    parser.add_argument("--episodes", type=int, default=1000)
    parser.add_argument("--output", type=str, default="")
    args = parser.parse_args()

    config = robotsim.Config()
    policies = [
        ("random", RandomPolicy(seed=0)),
        ("rule-based", RuleBasedPolicy(config)),
    ]

    lines = [
        "# robotsim baseline policies",
        "",
        f"- Episodes per policy: {args.episodes:,}",
        f"- Evaluation seeds: {EVAL_SEED_OFFSET:,} and above (never used for training)",
        f"- Map: fixed warehouse layout, {len(config.obstacles)} obstacles, "
        f"random start and goal at least {config.min_start_goal_distance:g} m apart",
        f"- Episode limit: {config.max_steps} steps",
        "",
        TABLE_HEADER,
    ]
    details = []
    for name, policy in policies:
        start = time.perf_counter()
        result = evaluate(policy, episodes=args.episodes)
        elapsed = time.perf_counter() - start
        lines.append(result.as_row(name))
        success_length = (
            f"{result.mean_success_length:.0f} steps on successful episodes"
            if result.mean_success_length == result.mean_success_length  # not NaN
            else "no successful episodes"
        )
        details.append(f"- {name}: {success_length}, evaluated in {elapsed:.1f} s")

    lines += ["", *details, "", f"- Commit: {git_commit()}", f"- Date: {time.strftime('%Y-%m-%d')}",
              "", "Reproduce with `python scripts/evaluate_baselines.py`."]

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