# Design Decisions

Each entry records a decision, its context and consequences.

---

## D-001: Disable Smart App Control on the development machine

**Context:** Windows 11 Smart App Control blocks unsigned executables without cloud reputation. This affected pip's launcher executables in `.venv\Scripts\` and, critically, locally compiled test binaries (`robotsim_tests.exe` could not be started by `ctest`). Self-signed certificates do not satisfy the policy.

**Alternatives considered:**
- Keep Smart App Control enabled and run C++ tests from inside the Python module (which was not blocked). Rejected: couples C++ tests to Python and the same issue would reappear for benchmark executables and the Unity native plugin.

**Decision:** Disable Smart App Control on the development machine. Real-time antivirus protection (Norton) and SmartScreen remain enabled. Only the `build/` directory is excluded from antivirus scanning, because every build produces new executables without reputation that were otherwise held for scanning and slowed down test runs.

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

---

## D-006: Exact unicycle integration, collision priority and tunnelling limit

**Context:** The project plan specified explicit Euler integration. Euler drifts outward on constant turns, and collisions are only checked at discrete poses.

**Decision:**
- Integrate the unicycle model exactly for constant `(v, omega)` over `dt`, falling back to a straight line when `|omega| < 1e-9`.
- If the goal is reached and a collision occurs in the same step, the step counts as a collision, not a success.
- `validate()` requires `v_max * dt <= robot_radius`, so the robot cannot pass through an obstacle between two collision checks.

**Consequences:** Turning trajectories match the analytic circle within 1e-9. Brief grazing contacts along the swept path can still go undetected; swept-volume checks are out of scope.

---

## D-007: Observation written into caller buffers; scope of determinism

**Context:** Phase 4 stores observations of many environments in one preallocated block. Trigonometric functions from different math libraries are not guaranteed to return correctly rounded results.

**Decision:**
- `World::write_observation(float*)` and `World::scan_lidar(double*)` write into buffers owned by the caller and do not allocate in the common case.
- Beam directions are precomputed in the robot frame; a scan evaluates one sin/cos pair for the heading.
- The goal bearing is encoded by rotating the goal vector into the robot frame, without `atan2`.
- Determinism is guaranteed bit for bit for the same binary. The random number generator is exact across platforms, but trajectories may differ in the last digit between compilers because `std::sin`/`std::cos` implementations differ.

**Consequences:** Unit tests avoid geometric configurations that depend on the last bit of a trigonometric result, such as rays aimed exactly at a box corner.
