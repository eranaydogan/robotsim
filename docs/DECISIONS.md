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

**Measurement (Phase 1 benchmark, 1,000,000 steps):** GCC 13 on Linux and MSVC 19.44 on Windows produced the same number of episodes (26,723), but the accumulated checksum differed in the last hexadecimal digit (`0x1.0835edd13906fp+24` versus `0x1.0835edd139067p+24`, about 3e-8). Episode outcomes matched; bit-level results did not. Differences between the math library trigonometric functions are the most likely cause.

---

## D-008: Benchmark methodology

**Context:** The first benchmark runs on a hybrid CPU (8 performance and 16 efficiency cores) varied by 45 percent between repetitions of identical work, while the checksum stayed the same. The scheduler moved the single benchmark thread between core types, and background applications made it worse.

**Decision:** Every benchmark runs one untimed warm-up followed by several timed repetitions, and reports the median with the minimum and maximum. `--pin` (the default in the runner script) pins the thread to a logical processor of the highest efficiency class reported by `GetLogicalProcessorInformationEx`. The report records CPU, power plan and mode, AC status and commit. A checksum of the simulated data is printed so that a change in speed can be separated from a change in behaviour.

**Consequences:** Three consecutive runs agreed within 0.2 percent. Peak throughput is slightly lower than an unpinned run, because the scheduler can no longer migrate the thread to the fastest available core; repeatability is worth more than the peak number. Reported figures are single-core and must not be extrapolated to the multi-threaded results of Phase 4.

---

## D-009: One source of truth for configuration, and seeding from the Gymnasium generator

**Context:** The plan called for a Python dataclass mirroring the C++ `Config`. Duplicated defaults drift apart. Gymnasium passes a seed to `reset` only on the first call, while episodes continue afterwards.

**Decision:**
- The C++ `Config` is exposed directly through pybind11; `robotsim.Config()` carries the same defaults as the C++ code, and a test asserts a few of them.
- `RobotNavEnv.reset` draws the 64-bit seed of the C++ environment from `self.np_random` on every reset.

**Consequences:** Defaults exist in exactly one place. A single seed reproduces the whole sequence of episodes, which a test verifies; the cost is one 64-bit draw per episode.

---

## D-010: Rendering with the Agg backend, matplotlib as an optional dependency

**Context:** Rendering is needed for debugging, for the render check in `check_env` and for the recordings of Phase 6, but not for training.

**Decision:** `Renderer` uses `Figure` and `FigureCanvasAgg` directly instead of `pyplot`: no global state, no GUI backend, and one figure reused across frames. It is imported lazily on the first `render()` call. matplotlib is an optional extra (`viz`); the render tests are skipped when it is missing and CI installs it so they run. The only render mode is `rgb_array`; a legend names the robot, goal, LiDAR, path and obstacles.

**Consequences:** Training does not import matplotlib. The trail shows only the steps that were rendered, which is documented on the class.

---

## D-011: Three seed pools and a tuned baseline

**Context:** A single training run can succeed by luck, and a weak baseline makes any agent look good. Model selection and reporting must not share episodes.

**Decision:**
- Three disjoint seed pools: training environments start from seed 0, the callback that monitors progress uses seeds from 1,500,000, and the reported evaluation uses seeds from 1,000,000.
- Every policy, baseline or agent, is measured with the same `robotsim.evaluate` protocol: 1,000 episodes, deterministic actions, success, collision and timeout reported separately.
- The rule-based controller's gains come from a small grid search, so PPO is compared against a tuned controller rather than a straw man. Tuning raised it from 52% to 73% success.
- Results are reported over three training seeds as mean and standard deviation.

**Consequences:** The reported figures are never measured on episodes used for training or model selection. Collision rate turned out to vary more across seeds (2.0% to 9.0%) than success rate, which is stated in the README rather than hidden behind the mean.

---

## D-012: Loading Stable-Baselines3 models through memory

**Context:** With torch 2.14 and Stable-Baselines3 2.9 on Windows, `PPO.load` fails with "PytorchStreamReader failed reading file .data/serialization_id". The archives are intact: the same members load correctly from a `BytesIO` or from an extracted file, and the identical versions work on Linux. Stable-Baselines3 passes torch an open member of the model archive.

**Decision:** `robotsim.sb3.load_ppo` wraps `torch.load` for the duration of the call so that file-like inputs are read into memory first, then restores the original function.

**Consequences:** Existing checkpoints load unchanged; saving is untouched. The workaround is scoped to one call and documented for removal once Stable-Baselines3 materialises archive members itself.
