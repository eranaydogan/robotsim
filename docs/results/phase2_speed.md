# robotsim Python interface benchmark

- Build: robotsim | MSVC 194435225 | C++ 201703 | 64-bit | Release
- Python 3.12.0, NumPy 2.5.3, Gymnasium 1.3.0
- Steps per repetition: 50,000
- Repetitions: 5 timed runs after one warm-up, single thread
- Thread pinned: logical processor 14 (efficiency class 1)

| Layer | Median /s | Min /s | Max /s |
|---|---|---|---|
| World.step | 1,211,751 | 1,198,098 | 1,213,430 |
| World.step + observation | 327,426 | 324,668 | 328,033 |
| RobotNavEnv.step | 133,959 | 133,522 | 134,683 |

## Machine

- CPU: 13th Gen Intel(R) Core(TM) i9-13900HX
- OS: Windows-11-10.0.26200-SP0
- Commit: 9069d8b
- Date: 2026-09-18

Reproduce with `python scripts/benchmark_env.py`.
