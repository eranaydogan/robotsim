"""Adapters for Stable-Baselines3 models.

Imported only where Stable-Baselines3 is available; the package itself is an
optional dependency of robotsim.
"""

from __future__ import annotations

import io
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator

import numpy as np


class SB3Policy:
    """Wraps a trained SB3 model so that it can be used with `robotsim.evaluate`."""

    def __init__(self, model: Any, deterministic: bool = True) -> None:
        self._model = model
        self._deterministic = deterministic

    def act(self, observation: np.ndarray) -> np.ndarray:
        action, _ = self._model.predict(observation, deterministic=self._deterministic)
        return np.asarray(action, dtype=np.float64)


@contextmanager
def _torch_load_from_memory() -> Iterator[None]:
    """Makes ``torch.load`` read file-like inputs fully into memory first.

    Stable-Baselines3 hands ``torch.load`` an open member of the model archive.
    With torch 2.14 and Stable-Baselines3 2.9 on Windows, reading a checkpoint
    from such a stream fails with "PytorchStreamReader failed reading file
    .data/serialization_id", although the same bytes load fine from memory or
    from a real file. Reading the member into a ``BytesIO`` first avoids it.

    Remove this once Stable-Baselines3 materialises archive members itself.
    """
    import torch

    original = torch.load

    def patched(f: Any, *args: Any, **kwargs: Any) -> Any:
        if hasattr(f, "read") and not isinstance(f, io.BytesIO):
            f = io.BytesIO(f.read())
        return original(f, *args, **kwargs)

    torch.load = patched
    try:
        yield
    finally:
        torch.load = original


def load_ppo(path: str | Path, device: str = "cpu") -> Any:
    """Loads a PPO model saved by `scripts/train.py`."""
    from stable_baselines3 import PPO

    with _torch_load_from_memory():
        return PPO.load(Path(path), device=device)