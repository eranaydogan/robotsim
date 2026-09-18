import numpy as np

import pytest

from robotsim.sb3 import SB3Policy, _torch_load_from_memory


class StubModel:
    """Minimal stand-in for an SB3 model: predict returns (action, state)."""

    def __init__(self) -> None:
        self.calls = []

    def predict(self, observation, deterministic=False):
        self.calls.append(deterministic)
        return np.array([0.25, -0.5], dtype=np.float32), None


def test_adapter_forwards_observations_and_returns_float64():
    model = StubModel()
    policy = SB3Policy(model)
    action = policy.act(np.zeros(40, dtype=np.float32))
    assert action.dtype == np.float64
    assert action.tolist() == [0.25, -0.5]
    assert model.calls == [True]


def test_adapter_can_sample_stochastically():
    model = StubModel()
    SB3Policy(model, deterministic=False).act(np.zeros(40, dtype=np.float32))
    assert model.calls == [False]


def test_torch_load_patch_is_scoped():
    torch = pytest.importorskip("torch")
    original = torch.load
    with _torch_load_from_memory():
        assert torch.load is not original
    assert torch.load is original
