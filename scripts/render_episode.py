"""Renders one episode of a trained model (or a baseline) to a PNG or GIF.

Examples:
    python scripts/render_episode.py --model runs/ppo-s0/final_model.zip \
        --outcome success --output docs/results/phase3_episode.png
    python scripts/render_episode.py --policy rule-based --output episode.gif
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image

import robotsim
from robotsim.baselines import RandomPolicy, RuleBasedPolicy
from robotsim.evaluate import EVAL_SEED_OFFSET

ROOT = Path(__file__).resolve().parent.parent


def load_policy(args: argparse.Namespace):
    if args.model:
        from robotsim.sb3 import SB3Policy, load_ppo

        path = Path(args.model)
        if not path.is_absolute():
            path = ROOT / path
        return SB3Policy(load_ppo(path))
    if args.policy == "rule-based":
        return RuleBasedPolicy(robotsim.Config())
    return RandomPolicy(seed=0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=str, default="", help="path to an SB3 .zip model")
    parser.add_argument("--policy", choices=["rule-based", "random"], default="rule-based")
    parser.add_argument("--outcome", choices=["success", "collision", "timeout", "any"],
                        default="success", help="keep searching seeds for this outcome")
    parser.add_argument("--seed", type=int, default=EVAL_SEED_OFFSET,
                        help="first evaluation seed to try")
    parser.add_argument("--max-seeds", type=int, default=50)
    parser.add_argument("--output", type=str, required=True, help=".png or .gif")
    parser.add_argument("--fps", type=int, default=10, help="frames per second for a GIF")
    args = parser.parse_args()

    policy = load_policy(args)
    env = robotsim.RobotNavEnv(render_mode="rgb_array")

    for attempt in range(args.max_seeds):
        observation, _ = env.reset(seed=args.seed + attempt)
        frames = [env.render()]
        while True:
            observation, _, terminated, truncated, info = env.step(policy.act(observation))
            frames.append(env.render())
            if terminated or truncated:
                break
        outcome = "success" if info["is_success"] else (
            "collision" if info["collision"] else "timeout"
        )
        if args.outcome in ("any", outcome):
            break
    else:
        print(f"no episode with outcome {args.outcome!r} in {args.max_seeds} seeds")
        return 1

    out = Path(args.output)
    if not out.is_absolute():
        out = ROOT / out
    out.parent.mkdir(parents=True, exist_ok=True)

    if out.suffix.lower() == ".gif":
        images = [Image.fromarray(frame) for frame in frames]
        images[0].save(out, save_all=True, append_images=images[1:],
                       duration=int(1000 / args.fps), loop=0)
    else:
        Image.fromarray(frames[-1]).save(out)

    print(f"{outcome} in {len(frames) - 1} steps, seed {args.seed + attempt}")
    print(f"written to {out} ({out.stat().st_size / 1024:.0f} kB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())