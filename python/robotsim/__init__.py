"""robotsim: C++ mobile robot navigation environment with a Gymnasium interface."""

from importlib.metadata import PackageNotFoundError, version

from ._robotsim import build_info

try:
    __version__ = version("robotsim")
except PackageNotFoundError:
    __version__ = "0.0.0"

__all__ = ["build_info", "__version__"]