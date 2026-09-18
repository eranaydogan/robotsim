# robotsim C++ core benchmark

- Build: robotsim | MSVC 194435225 | C++ 201703 | 64-bit | Release
- Hardware threads: 32
- Configuration: default (36 LiDAR beams, 10 obstacles + 4 walls)
- Repetitions: 5 timed runs after one warm-up, single thread
- Thread pinned: logical processor 14 (efficiency class 1)

| Benchmark                  |  Iterations |    Median /s |       Min /s |       Max /s |
|----------------------------|-------------|--------------|--------------|--------------|
| reset                      |     1000000 |      8530443 |      8474706 |      8540410 |
| step                       |     5000000 |     15189803 |     15167970 |     15304790 |
| scan_lidar                 |     1000000 |       685122 |       675964 |       694314 |
| step + write_observation   |     1000000 |       518803 |       518746 |       519305 |

- Episodes in step + observation workload: 26723
- Checksum of rewards and observations: 17315309.817276 (0x1.0835edd139067p+24)

## Machine

- CPU: 13th Gen Intel(R) Core(TM) i9-13900HX
- Cores / logical processors: 24 / 32
- RAM: 31.7 GB
- OS: Microsoft Windows 11 Home Single Language
- Power plan: Dengeli; power mode (AC): Best performance; AC power: Online
- Note: Monster Control Center profile: Performance
- Commit: 6307b6b
- Date: 2026-09-18

Reproduce with `.\scripts\run_cpp_benchmark.ps1`.
