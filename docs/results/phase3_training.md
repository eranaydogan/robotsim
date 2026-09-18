# robotsim Phase 3 training results

- Algorithm: PPO (Stable-Baselines3), 8 parallel environments, 2,000,000 steps per run
- Seeds: 0, 1, 2
- Evaluation: 1,000 episodes per policy on seeds 1,000,000 and above, deterministic actions
- Map: fixed warehouse layout, 10 obstacles, episode limit 500 steps

| Policy | Success | Collision | Timeout | Mean return | Mean length |
|---|---|---|---|---|---|
| random | 0.3% | 99.7% | 0.0% | -10.44 | 37.8 |
| rule-based | 73.1% | 22.4% | 4.5% | 7.86 | 173.4 |
| ppo-s0 | 89.1% | 5.7% | 5.2% | 12.44 | 93.5 |
| ppo-s1 | 83.3% | 9.0% | 7.7% | 11.23 | 103.7 |
| ppo-s2 | 90.3% | 2.0% | 7.7% | 12.73 | 107.2 |
| **PPO (3 seeds)** | **87.6% ± 3.7%** | **5.6% ± 3.5%** | 6.9% ± 1.4% | | |

## Training

- Wall-clock time per run: 3.1 min (10,673 environment steps per second)
- Hardware: Windows-11-10.0.26200-SP0
- Build: robotsim | MSVC 194435225 | C++ 201703 | 64-bit | Release
- Commit: 0f7758e (with uncommitted changes)
- Date: 2026-09-18

Reproduce with `python scripts/train.py --seed <n> --name ppo-s<n>` followed by `python scripts/collect_results.py --runs ppo-s0 ppo-s1 ppo-s2`.
