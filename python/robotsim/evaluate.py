"""Shared evaluation protocol for baselines and trained agents."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np

from .env import RobotNavEnv

# Seeds below this value are reserved for training; evaluation uses seeds above
# it, so that reported numbers never come from episodes seen during training.
EVAL_SEED_OFFSET = 1_000_000


class Actor(Protocol):
    def act(self, observation: np.ndarray) -> np.ndarray: ...


@dataclass
class EvalResult:
    episodes: int
    success_rate: float
    collision_rate: float
    timeout_rate: float
    mean_return: float
    mean_length: float
    mean_success_length: float
    returns: np.ndarray = field(repr=False)

    def as_row(self, name: str) -> str:
        return (
            f"| {name} | {self.success_rate:.1%} | {self.collision_rate:.1%} | "
            f"{self.timeout_rate:.1%} | {self.mean_return:.2f} | {self.mean_length:.1f} |"
        )


TABLE_HEADER = (
    "| Policy | Success | Collision | Timeout | Mean return | Mean length |\n"
    "|---|---|---|---|---|---|"
)


def evaluate(
    policy: Actor,
    env: RobotNavEnv | None = None,
    episodes: int = 1000,
    seed_offset: int = EVAL_SEED_OFFSET,
    on_episode: Callable[[int, str], None] | None = None,
) -> EvalResult:
    """Runs `episodes` episodes on held-out seeds and returns aggregate metrics."""
    env = env if env is not None else RobotNavEnv()

    successes = collisions = timeouts = 0
    returns = np.zeros(episodes)
    lengths = np.zeros(episodes)
    success_lengths: list[int] = []

    for episode in range(episodes):
        observation, _ = env.reset(seed=seed_offset + episode)
        if hasattr(policy, "reset"):
            policy.reset()

        total = 0.0
        while True:
            observation, reward, terminated, truncated, info = env.step(
                policy.act(observation)
            )
            total += reward
            if terminated or truncated:
                break

        outcome = "success" if info["is_success"] else (
            "collision" if info["collision"] else "timeout"
        )
        successes += outcome == "success"
        collisions += outcome == "collision"
        timeouts += outcome == "timeout"
        returns[episode] = total
        lengths[episode] = env.world.step_count
        if outcome == "success":
            success_lengths.append(env.world.step_count)
        if on_episode is not None:
            on_episode(episode, outcome)

    return EvalResult(
        episodes=episodes,
        success_rate=successes / episodes,
        collision_rate=collisions / episodes,
        timeout_rate=timeouts / episodes,
        mean_return=float(returns.mean()),
        mean_length=float(lengths.mean()),
        mean_success_length=float(np.mean(success_lengths)) if success_lengths else float("nan"),
        returns=returns,
    )