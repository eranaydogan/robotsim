import numpy as np
import pytest

import robotsim
from robotsim.baselines import RandomPolicy, RuleBasedPolicy
from robotsim.evaluate import EVAL_SEED_OFFSET, evaluate


def test_policies_return_valid_actions():
    env = robotsim.RobotNavEnv()
    observation, _ = env.reset(seed=0)
    for policy in (RandomPolicy(0), RuleBasedPolicy(env.config)):
        for _ in range(50):
            action = policy.act(observation)
            assert action.shape == (2,)
            assert np.all(np.abs(action) <= 1.0)
            observation, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                observation, _ = env.reset()


def test_evaluate_rates_are_consistent():
    result = evaluate(RandomPolicy(0), episodes=20)
    assert result.episodes == 20
    total = result.success_rate + result.collision_rate + result.timeout_rate
    assert total == pytest.approx(1.0)
    assert result.returns.shape == (20,)


def test_evaluate_is_reproducible():
    first = evaluate(RuleBasedPolicy(robotsim.Config()), episodes=20)
    second = evaluate(RuleBasedPolicy(robotsim.Config()), episodes=20)
    assert np.array_equal(first.returns, second.returns)
    assert first.success_rate == second.success_rate


def test_evaluation_seeds_are_held_out():
    # Training uses seeds below the offset; evaluation must not overlap.
    assert EVAL_SEED_OFFSET >= 1_000_000


def test_rule_based_beats_random():
    config = robotsim.Config()
    rule = evaluate(RuleBasedPolicy(config), episodes=50)
    random = evaluate(RandomPolicy(0), episodes=50)
    assert rule.success_rate > random.success_rate
    assert rule.collision_rate < random.collision_rate