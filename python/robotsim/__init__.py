"""robotsim: C++ mobile robot navigation environment with a Gymnasium interface."""

from importlib.metadata import PackageNotFoundError, version

from gymnasium.envs.registration import register

from ._robotsim import Config, StepResult, World, build_info
from .env import RobotNavEnv

try:
    __version__ = version("robotsim")
except PackageNotFoundError:
    __version__ = "0.0.0"

register(id="RobotNav-v0", entry_point="robotsim.env:RobotNavEnv")

__all__ = [
    "Config",
    "RobotNavEnv",
    "StepResult",
    "World",
    "build_info",
    "__version__",
]