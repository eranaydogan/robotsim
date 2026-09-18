import warnings

import gymnasium as gym
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

import robotsim


def make_env() -> robotsim.RobotNavEnv:
    return robotsim.RobotNavEnv()


def test_check_env_without_warnings():
    env = make_env()
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        check_env(env, skip_render_check=True)


def test_registered_in_gymnasium():
    env = gym.make("RobotNav-v0")
    assert isinstance(env.unwrapped, robotsim.RobotNavEnv)


def test_spaces_match_the_configuration():
    env = make_env()
    beams = env.config.lidar_beams
    assert env.observation_space.shape == (beams + 4,)
    assert env.observation_space.dtype == np.float32
    assert env.action_space.shape == (2,)
    # The bearing components are the only observations that can be negative.
    assert np.all(env.observation_space.low[: beams + 1] == 0.0)
    assert np.all(env.observation_space.low[beams + 1 : beams + 3] == -1.0)
    assert env.observation_space.low[beams + 3] == 0.0


def test_random_rollout_stays_valid():
    env = make_env()
    obs, info = env.reset(seed=3)
    assert obs in env.observation_space
    assert info["is_success"] is False

    rng = np.random.default_rng(3)
    episodes = 0
    for _ in range(100_000):
        obs, reward, terminated, truncated, info = env.step(rng.uniform(-1.0, 1.0, size=2))
        assert obs in env.observation_space, "observation outside the declared space"
        assert np.isfinite(obs).all()
        assert np.isfinite(reward)
        assert not (terminated and truncated)
        if terminated or truncated:
            episodes += 1
            obs, info = env.reset()
    assert episodes > 100


def test_same_seed_gives_the_same_episode():
    actions = np.random.default_rng(0).uniform(-1.0, 1.0, size=(200, 2))

    def rollout(seed):
        env = make_env()
        obs, _ = env.reset(seed=seed)
        trace = [obs.copy()]
        rewards = []
        for action in actions:
            obs, reward, terminated, truncated, _ = env.step(action)
            trace.append(obs.copy())
            rewards.append(reward)
            if terminated or truncated:
                break
        return np.array(trace), np.array(rewards)

    obs_a, rew_a = rollout(7)
    obs_b, rew_b = rollout(7)
    obs_c, _ = rollout(8)

    assert np.array_equal(obs_a, obs_b)
    assert np.array_equal(rew_a, rew_b)
    assert obs_a.shape != obs_c.shape or not np.array_equal(obs_a, obs_c)


def test_seeded_reset_reproduces_the_whole_sequence():
    def first_positions(seed, episodes=5):
        env = make_env()
        env.reset(seed=seed)
        positions = [env.world.position]
        for _ in range(episodes - 1):
            env.reset()
            positions.append(env.world.position)
        return positions

    assert first_positions(11) == first_positions(11)
    assert first_positions(11) != first_positions(12)


def test_action_scaling():
    env = make_env()
    cfg = env.config
    env.reset(seed=0)

    env.world.set_episode(x=5.0, y=5.0, heading=0.0, goal_x=9.0, goal_y=9.0)
    env.step(np.array([1.0, 1.0], dtype=np.float32))
    assert env.world.v == pytest.approx(cfg.v_max)
    assert env.world.omega == pytest.approx(cfg.omega_max)

    env.world.set_episode(x=5.0, y=5.0, heading=0.0, goal_x=9.0, goal_y=9.0)
    env.step(np.array([-1.0, -1.0], dtype=np.float32))
    assert env.world.v == pytest.approx(0.0)
    assert env.world.omega == pytest.approx(-cfg.omega_max)

    env.world.set_episode(x=5.0, y=5.0, heading=0.0, goal_x=9.0, goal_y=9.0)
    env.step(np.array([0.0, 0.0], dtype=np.float32))
    assert env.world.v == pytest.approx(0.5 * cfg.v_max)
    assert env.world.omega == pytest.approx(0.0)


def test_actions_outside_the_space_are_clipped():
    env = make_env()
    cfg = env.config
    env.reset(seed=0)
    env.world.set_episode(x=5.0, y=5.0, heading=0.0, goal_x=9.0, goal_y=9.0)
    env.step(np.array([10.0, -10.0]))
    assert env.world.v == pytest.approx(cfg.v_max)
    assert env.world.omega == pytest.approx(-cfg.omega_max)


def test_wrong_action_shape_raises():
    env = make_env()
    env.reset(seed=0)
    with pytest.raises(ValueError):
        env.step(np.array([0.0, 0.0, 0.0]))


def test_goal_and_collision_are_reported():
    env = make_env()
    env.reset(seed=0)

    # Goal 0.2 m ahead, well inside the goal radius after one step.
    env.world.set_episode(x=5.0, y=1.0, heading=0.0, goal_x=5.2, goal_y=1.0)
    _, reward, terminated, truncated, info = env.step(np.array([1.0, 0.0]))
    assert terminated and not truncated
    assert info["is_success"] is True
    assert info["collision"] is False
    assert reward > env.config.reward_goal - 1.0

    # Facing the left wall from 0.25 m away.
    env.world.set_episode(x=0.25, y=5.0, heading=np.pi, goal_x=9.0, goal_y=5.0)
    _, reward, terminated, _, info = env.step(np.array([1.0, 0.0]))
    assert terminated
    assert info["collision"] is True
    assert info["is_success"] is False
    assert reward < -env.config.reward_collision + 1.0


def test_truncation_after_max_steps():
    cfg = robotsim.Config()
    cfg.max_steps = 12
    env = robotsim.RobotNavEnv(cfg)
    env.reset(seed=1)
    env.world.set_episode(x=5.0, y=1.0, heading=0.0, goal_x=9.5, goal_y=9.5)
    for _ in range(cfg.max_steps - 1):
        _, _, terminated, truncated, _ = env.step(np.array([-1.0, 0.0]))
        assert not terminated and not truncated
    _, _, terminated, truncated, _ = env.step(np.array([-1.0, 0.0]))
    assert truncated and not terminated


def test_invalid_config_raises():
    cfg = robotsim.Config()
    cfg.robot_radius = -1.0
    with pytest.raises(ValueError):
        robotsim.RobotNavEnv(cfg)


def test_config_defaults_come_from_cpp():
    cfg = robotsim.Config()
    assert cfg.lidar_beams == 36
    assert len(cfg.obstacles) == 10
    assert cfg.obstacles[0] == pytest.approx((1.5, 2.0, 4.0, 2.5))