# robotsim baseline policies

- Episodes per policy: 1,000
- Evaluation seeds: 1,000,000 and above (never used for training)
- Map: fixed warehouse layout, 10 obstacles, random start and goal at least 2 m apart
- Episode limit: 500 steps

| Policy | Success | Collision | Timeout | Mean return | Mean length |
|---|---|---|---|---|---|
| random | 0.3% | 99.7% | 0.0% | -10.44 | 37.8 |
| rule-based | 73.1% | 22.4% | 4.5% | 7.86 | 173.4 |

- random: 50 steps on successful episodes, evaluated in 0.3 s
- rule-based: 176 steps on successful episodes, evaluated in 3.5 s

- Commit: dbd996a (with uncommitted changes)
- Date: 2026-09-18

Reproduce with `python scripts/evaluate_baselines.py`.
