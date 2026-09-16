# robotsim

[![CI](https://github.com/eranaydogan/robotsim/actions/workflows/ci.yml/badge.svg)](https://github.com/eranaydogan/robotsim/actions/workflows/ci.yml)

C++17 mobile robot navigation environment with a Gymnasium-compatible Python interface.

**Status:** Phase 0 complete (build skeleton, tests, CI). See the [project plan](docs/PROJECT_PLAN.md).

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

# Python package (editable) and tests
python -m pip install -e . --no-build-isolation
python -m pytest
```

After changing C++ code, rerun the `pip install` command to rebuild the module.

## Documentation

- [Project plan](docs/PROJECT_PLAN.md)
- [Design decisions](docs/DECISIONS.md)