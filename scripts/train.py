"""Trains a PPO agent on RobotNav-v0 and evaluates it on held-out seeds.

Example:
    python scripts/train.py --timesteps 2000000 --seed 0 --name ppo-s0

Everything needed to reproduce a run is written to ``runs/<name>/``: the
arguments, the git commit, the library versions, TensorBoard logs, the final
model and the evaluation result.
"""

from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

import robotsim
from robotsim.env import RobotNavEnv
from robotsim.evaluate import EVAL_SEED_OFFSET, TABLE_HEADER, evaluate
from robotsim.sb3 import SB3Policy

ROOT = Path(__file__).resolve().parent.parent

# Seeds used by the callback that monitors progress during training. They are
# held out from training but kept apart from the final evaluation seeds, so the
# reported result is never measured on episodes used for model selection.
MONITOR_SEED_OFFSET = EVAL_SEED_OFFSET + 500_000


def git_commit() -> str:
    try:
        commit = subprocess.run(
            ["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "-C", str(ROOT), "status", "--porcelain"],
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        return commit + (" (with uncommitted changes)" if dirty else "")
    except Exception:
        return "unknown"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timesteps", type=int, default=2_000_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--envs", type=int, default=8, help="parallel environments")
    parser.add_argument("--name", type=str, default="", help="run name (default: ppo-s<seed>)")
    parser.add_argument("--threads", type=int, default=1, help="torch CPU threads")
    parser.add_argument("--eval-episodes", type=int, default=1000)
    parser.add_argument("--eval-freq", type=int, default=100_000,
                        help="monitoring evaluation every N environment steps")
    parser.add_argument("--n-steps", type=int, default=512, help="PPO rollout length per env")
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--ent-coef", type=float, default=0.005)
    parser.add_argument("--gamma", type=float, default=0.99)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    name = args.name or f"ppo-s{args.seed}"
    run_dir = ROOT / "runs" / name
    run_dir.mkdir(parents=True, exist_ok=True)

    # A small MLP on a 40-dimensional observation: more threads only add
    # synchronisation overhead.
    torch.set_num_threads(args.threads)

    train_env = make_vec_env(RobotNavEnv, n_envs=args.envs, seed=args.seed)
    monitor_env = Monitor(RobotNavEnv())
    monitor_env.reset(seed=MONITOR_SEED_OFFSET + args.seed)

    model = PPO(
        "MlpPolicy",
        train_env,
        seed=args.seed,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        ent_coef=args.ent_coef,
        gamma=args.gamma,
        verbose=1,
        device="cpu",
        tensorboard_log=str(run_dir / "tb"),
    )

    callback = EvalCallback(
        monitor_env,
        best_model_save_path=str(run_dir),
        log_path=str(run_dir),
        eval_freq=max(args.eval_freq // args.envs, 1),
        n_eval_episodes=50,
        deterministic=True,
        verbose=1,
    )

    metadata = {
        "args": vars(args),
        "commit": git_commit(),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "build_info": robotsim.build_info(),
        "versions": {
            "torch": torch.__version__,
            "numpy": np.__version__,
        },
        "started": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    start = time.perf_counter()
    model.learn(total_timesteps=args.timesteps, callback=callback, progress_bar=False)
    train_seconds = time.perf_counter() - start
    model.save(run_dir / "final_model")

    # Final evaluation of the last model on the held-out seeds.
    result = evaluate(SB3Policy(model), episodes=args.eval_episodes)

    summary = {
        **metadata,
        "train_seconds": round(train_seconds, 1),
        "steps_per_second": round(args.timesteps / train_seconds, 1),
        "final": {
            "episodes": result.episodes,
            "success_rate": result.success_rate,
            "collision_rate": result.collision_rate,
            "timeout_rate": result.timeout_rate,
            "mean_return": result.mean_return,
            "mean_length": result.mean_length,
        },
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print()
    print(f"training time: {train_seconds / 60:.1f} min "
          f"({args.timesteps / train_seconds:,.0f} steps/s)")
    print(TABLE_HEADER)
    print(result.as_row(name))
    print(f"results written to {run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())