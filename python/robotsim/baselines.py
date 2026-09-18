"""Reference policies to compare a trained agent against.

Both policies see exactly the same observation as the agent: normalised LiDAR
ranges and the goal bearing, never the true pose. Anything else would make the
comparison unfair.
"""

from __future__ import annotations

import numpy as np

from ._robotsim import Config


class Policy:
    """Maps an observation to an action in [-1, 1]^2."""

    def reset(self) -> None:
        """Called at the start of an episode."""

    def act(self, observation: np.ndarray) -> np.ndarray:
        raise NotImplementedError


class RandomPolicy(Policy):
    """Uniform random actions: the lower bound for any agent."""

    def __init__(self, seed: int = 0) -> None:
        self._rng = np.random.default_rng(seed)

    def act(self, observation: np.ndarray) -> np.ndarray:
        return self._rng.uniform(-1.0, 1.0, size=2)


class RuleBasedPolicy(Policy):
    """Turn towards the goal, steer away from whatever the front beams see.

    The steering command combines two terms:

    * a pull towards the goal, from the sine of the goal bearing
    * a push away from obstacles, from the difference between the free space on
      the left and on the right within a forward-looking sector

    The linear velocity is reduced when the closest front beam is near, so the
    robot slows down before it has to turn.

    The default gains come from a small grid search over the sector width, the
    braking distance and the two gains, so that the agent is compared against a
    tuned controller rather than a straw man.
    """

    def __init__(
        self,
        config: Config,
        front_sector_deg: float = 90.0,
        brake_distance: float = 1.5,
        avoid_gain: float = 4.0,
        goal_gain: float = 1.5,
    ) -> None:
        self._beams = config.lidar_beams
        self._range = config.lidar_range
        self._brake = brake_distance / config.lidar_range
        self._avoid_gain = avoid_gain
        self._goal_gain = goal_gain

        # Beam i points at i * 360 / beams degrees, counter-clockwise from the
        # heading, so beams near 0 and near `beams` both look forward.
        angles_deg = np.arange(self._beams) * (360.0 / self._beams)
        angles_deg = (angles_deg + 180.0) % 360.0 - 180.0
        self._front = np.abs(angles_deg) <= front_sector_deg
        self._left = self._front & (angles_deg > 0.0)
        self._right = self._front & (angles_deg < 0.0)

    def act(self, observation: np.ndarray) -> np.ndarray:
        lidar = observation[: self._beams]
        sin_bearing = float(observation[self._beams + 1])
        cos_bearing = float(observation[self._beams + 2])

        front_clearance = float(lidar[self._front].min())
        left_clearance = float(lidar[self._left].mean())
        right_clearance = float(lidar[self._right].mean())

        # Positive omega turns left. Blocked on the right -> turn left.
        avoid = self._avoid_gain * (left_clearance - right_clearance)
        # Reverse the goal pull when the goal is behind: turning is then better
        # than driving forward, and the sine alone vanishes at +-180 degrees.
        goal_pull = self._goal_gain * sin_bearing
        if cos_bearing < 0.0:
            goal_pull = self._goal_gain * np.sign(sin_bearing or 1.0)

        # Obstacle avoidance dominates once the way ahead is blocked.
        blocked = np.clip(1.0 - front_clearance / self._brake, 0.0, 1.0)
        omega = (1.0 - blocked) * goal_pull + blocked * avoid
        if front_clearance < 0.3 * self._brake and abs(avoid) < 1e-3:
            # Dead ahead into a wall with symmetric clearance: pick a side.
            omega = 1.0

        # Slow down near obstacles; the action maps [-1, 1] to [0, v_max].
        speed = np.clip(front_clearance / self._brake, 0.15, 1.0)
        v_action = 2.0 * speed - 1.0

        return np.array([v_action, float(np.clip(omega, -1.0, 1.0))], dtype=np.float64)