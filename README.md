# robotsim

[![CI](https://github.com/eranaydogan/robotsim/actions/workflows/ci.yml/badge.svg)](https://github.com/eranaydogan/robotsim/actions/workflows/ci.yml)

C++17 mobile robot navigation environment with a Gymnasium-compatible Python interface.

**Status:** Phase 1 complete (C++ simulation core). See the [project plan](docs/PROJECT_PLAN.md).

## Simulation core

- Differential-drive robot with exact unicycle integration
- Circle-versus-box collision checking against shelves, pillars and walls
- 36-beam raycast LiDAR and a normalized 40-element observation
- Progress-based reward with goal bonus and collision penalty
- Portable xoshiro256** random number generator; bit-for-bit deterministic for a given binary

## Phase 1 results

Single thread pinned to one performance core, Intel Core i9-13900HX, MSVC 19.44, Release build:

| Workload | Steps per second |
|---|---|
| step + observation (36 LiDAR beams) | **518,803** |

The target was 100,000 steps per second. Three consecutive runs agreed within 0.2 percent.
Full report: [docs/results/phase1_speed.md](docs/results/phase1_speed.md).

## Requirements

- Windows 10/11 x64
- Visual Studio 2022 with the *Desktop development with C++* workload
- Python 3.12 (64-bit)

## Build and test

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install scikit-build-core pybind11 cmake numpy gymnasium pytest

# C++ unit tests
cmake -S . -B build/tests -DBUILD_TESTING=ON
cmake --build build/tests --config Release
ctest --test-dir build/tests -C Release --output-on-failure

# C++ benchmark and report
.\scripts\run_cpp_benchmark.ps1

# Python package (editable) and tests
python -m pip install -e . --no-build-isolation
python -m pytest
```

After changing C++ code, rerun the `pip install` command to rebuild the module.

## Documentation

- [Project plan](docs/PROJECT_PLAN.md)
- [Design decisions](docs/DECISIONS.md)