# Design Decisions

Each entry records a decision, its context and consequences.

---

## D-001: Disable Smart App Control on the development machine

**Context:** Windows 11 Smart App Control blocks unsigned executables without cloud reputation. This affected pip's launcher executables in `.venv\Scripts\` and, critically, locally compiled test binaries (`robotsim_tests.exe` could not be started by `ctest`). Self-signed certificates do not satisfy the policy.

**Alternatives considered:**
- Keep Smart App Control enabled and run C++ tests from inside the Python module (which was not blocked). Rejected: couples C++ tests to Python and the same issue would reappear for benchmark executables and the Unity native plugin.

**Decision:** Disable Smart App Control on the development machine. Microsoft Defender antivirus, firewall and SmartScreen remain enabled.

**Consequences:** Locally built executables and DLLs run normally. Python tools are still invoked as modules (`python -m pip`, `python -m pytest`), which works regardless of this setting.

---

## D-002: Build C++ tests and the Python module separately

**Context:** The Python module requires Python development headers and pybind11; the C++ unit tests do not.

**Decision:** A plain CMake configure builds the core library and tests only. The Python module is built when CMake is invoked by scikit-build-core (`pip install`), or explicitly with `-DROBOTSIM_BUILD_PYTHON=ON`.

**Consequences:** C++ tests can run without a Python environment. Two build directories are used: `build/tests` and `build/<wheel_tag>`.

---

## D-003: CI uses the latest Windows runner image

**Context:** Local development uses Visual Studio 2022 (MSVC 19.44). The `windows-latest` GitHub runner ships Visual Studio 2026 (MSVC 19.51).

**Decision:** Keep `windows-latest` instead of pinning an older image. CI builds with `/WX`, so every push also verifies that the code is warning-free on a newer MSVC release.

**Consequences:** A new compiler warning in CI may appear before it appears locally. Such failures are treated as real issues. CMake definitions for the Python build are passed via `SKBUILD_CMAKE_DEFINE`, because PowerShell (the default shell on Windows runners) splits `-C` arguments that contain dots.

---

## D-004: Double precision for simulation state, float32 for observations

**Context:** Robot poses are integrated over up to 500 steps, and unit tests compare trajectories against analytic solutions. Neural network inputs are conventionally float32.

**Decision:** Geometry and simulation state use `double`. Only the observation vector exported to Python is `float32`.

**Consequences:** Kinematics tests can use a tolerance of 1e-9 instead of 1e-5. The conversion cost for a 40-element observation is negligible. Axis-aligned boxes name their corners `lo`/`hi` to avoid clashes with the `min`/`max` macros from Windows headers.

---

## D-005: Boundary walls as boxes and seeded versus unseeded reset

**Context:** Collision checks and LiDAR raycasts must treat the map boundary like any other obstacle. Gymnasium passes a seed only on the first `reset`, and Phase 4 auto-resets finished environments without a seed.

**Decision:**
- `World` appends four 1 m thick boxes just outside the map to the obstacle list.
- `World::reset(seed)` restarts the random stream; `World::reset()` continues it.
- Start and goal are placed by rejection sampling with a bounded number of attempts; impossible configurations throw instead of looping forever.

**Consequences:** A single code path handles walls and obstacles. Episode sequences are reproducible from one initial seed.
