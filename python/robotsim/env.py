"""Gymnasium environment wrapping the C++ simulation core."""

from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from ._robotsim import Config, World

# Actions and observations are float32, the usual dtype for neural network input.
_DTYPE = np.float32

# Actions are sampled in [-1, 1] and scaled to physical units inside step().
_ACTION_LOW = -1.0
_ACTION_HIGH = 1.0


class RobotNavEnv(gym.Env):
    """Differential-drive robot navigating to a goal without collisions.

    Observation (``lidar_beams + 4`` values, all normalised):
        ``[0, N)``  LiDAR ranges divided by the sensor range
        ``N``       distance to the goal divided by the map diagonal
        ``N + 1``   sine of the goal bearing relative to the heading
        ``N + 2``   cosine of the goal bearing relative to the heading
        ``N + 3``   last applied linear velocity divided by ``v_max``

    Action (2 values in ``[-1, 1]``):
        ``a[0]``    linear velocity, mapped to ``[0, v_max]``
        ``a[1]``    angular velocity, mapped to ``[-omega_max, omega_max]``

    An episode ends when the goal is reached or the robot touches an obstacle
    (``terminated``), or after ``max_steps`` steps (``truncated``).
    """

    metadata = {"render_modes": [], "render_fps": 10}

    def __init__(self, config: Config | None = None, render_mode: str | None = None) -> None:
        super().__init__()
        self._world = World(config if config is not None else Config())
        cfg = self._world.config
        self.render_mode = render_mode

        size = self._world.observation_size
        # LiDAR and distance are in [0, 1]; the bearing components are in [-1, 1].
        obs_low = np.zeros(size, dtype=_DTYPE)
        obs_high = np.ones(size, dtype=_DTYPE)
        obs_low[cfg.lidar_beams + 1 : cfg.lidar_beams + 3] = -1.0
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=_DTYPE)
        self.action_space = spaces.Box(
            low=_ACTION_LOW, high=_ACTION_HIGH, shape=(2,), dtype=_DTYPE
        )

    @property
    def world(self) -> World:
        """The underlying C++ environment, for rendering and debugging."""
        return self._world

    @property
    def config(self) -> Config:
        return self._world.config

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        # Draw the C++ seed from the Gymnasium generator, so a single seed passed
        # here reproduces the whole sequence of episodes.
        world_seed = int(self.np_random.integers(0, 2**64, dtype=np.uint64))
        self._world.reset(world_seed)
        return self._world.observation(), self._info(is_success=False, collision=False)

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape != (2,):
            raise ValueError(f"expected an action of shape (2,), got {action.shape}")
        action = np.clip(action, _ACTION_LOW, _ACTION_HIGH)

        cfg = self._world.config
        v = float((action[0] + 1.0) * 0.5 * cfg.v_max)
        omega = float(action[1] * cfg.omega_max)

        result = self._world.step(v, omega)
        return (
            self._world.observation(),
            float(result.reward),
            bool(result.terminated),
            bool(result.truncated),
            self._info(is_success=result.is_success, collision=result.collision),
        )

    def _info(self, *, is_success: bool, collision: bool) -> dict[str, Any]:
        return {
            "is_success": bool(is_success),
            "collision": bool(collision),
            "position": self._world.position,
            "heading": self._world.heading,
            "goal": self._world.goal,
        }