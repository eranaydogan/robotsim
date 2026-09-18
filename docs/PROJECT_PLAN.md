# robotsim — Project Plan

**A C++ mobile robot navigation environment with a Gymnasium-compatible interface**

Platform: **Windows 10/11 x64 + MSVC (Visual Studio 2022)**

---

## Contents

1. [Goal and scope](#1-goal-and-scope)
2. [Timeline and overview](#2-timeline-and-overview)
3. [Technical rules and conventions](#3-technical-rules-and-conventions)
4. [Repository layout](#4-repository-layout)
5. [Phase 0 — Setup and skeleton](#phase-0--setup-and-skeleton)
6. [Phase 1 — C++ simulation core](#phase-1--c-simulation-core)
7. [Phase 2 — Gymnasium interface](#phase-2--gymnasium-interface)
8. [Phase 3 — First training run (MVP)](#phase-3--first-training-run-mvp)
9. [Phase 4 — Native vectorization and benchmarks](#phase-4--native-vectorization-and-benchmarks)
10. [Phase 5 — Realism](#phase-5--realism)
11. [Phase 6 — Visualization](#phase-6--visualization)
12. [Phase 7 — Documentation](#phase-7--documentation)
13. [Risks and mitigations](#13-risks-and-mitigations)
14. [Windows-specific pitfalls](#14-windows-specific-pitfalls)
15. [Progress tracking](#15-progress-tracking)

---

## 1. Goal and scope

**Goal:** Build an RL environment in which a differential-drive robot learns to reach a goal in a 2D warehouse without colliding with obstacles, using LiDAR-like sensing. The simulation core is written in C++, the interface follows the Gymnasium standard, and performance and realism are documented with measurements.

**In scope**
- C++17 simulation core (kinematics, collision checking, raycasting, reward, termination)
- Python bindings via pybind11 and a `gymnasium.Env` wrapper
- Training and evaluation with Stable-Baselines3 PPO
- Multi-threaded vectorized environment in C++ and benchmarks
- Sensor noise, domain randomization, procedural maps, moving obstacles
- Visualization via matplotlib GIFs and a Unity replay

**Out of scope (deliberately)**
- 3D physics, camera/pixel observations
- GPU training (CPU PyTorch is used)
- Developing new RL algorithms
- Unreal Engine integration (listed as a planned frontend)

**Definition of done for every phase**
- [ ] C++ and Python tests pass
- [ ] Code is pushed to GitHub and the phase is tagged (e.g. `v0.1-phase0`)
- [ ] The README section for the phase is up to date
- [ ] Important decisions are recorded in `docs/DECISIONS.md`

---

## 2. Timeline and overview

| Phase | Content | Duration | Output |
|---|---|---|---|
| 0 | Setup and skeleton | 1 day | C++ function callable from Python |
| 1 | C++ simulation core | 2–3 days | Tested, deterministic simulator |
| 2 | Gymnasium interface | 1–2 days | Environment passing `check_env` |
| 3 | First training run (MVP) | 2–3 days | Agent that learns the task |
| 4 | Native vectorization and benchmarks | 3–4 days | steps/s comparison table |
| 5 | Realism | 3–5 days | Ablation and robustness results |
| 6 | Visualization | 2–3 days | GIFs, Unity replay |
| 7 | Documentation | 1 day | Final README |

Durations assume focused work. **The critical path is Phases 0–3.**

Dependencies:

```
Phase 0 → Phase 1 → Phase 2 → Phase 3 (MVP)
                        │
                        └──→ Phase 4 (requires Phase 2 only)
Phase 3 + Phase 4 → Phase 5 → Phase 6 → Phase 7
```

---

## 3. Technical rules and conventions

These rules apply to all phases. If something needs to change, update this section first.

### 3.1 Toolchain

| Component | Choice | Notes |
|---|---|---|
| OS | Windows 10/11 x64 | |
| Compiler | MSVC (VS 2022, v143) | VS Community includes the profiler |
| C++ standard | C++17 (`/std:c++17`) | |
| Build system | CMake ≥ 3.20 | Installed via `pip install cmake` |
| Python | 3.12, **64-bit** | The compiled module is tied to this version |
| Bindings | pybind11 | Header-only, installed via pip |
| Packaging | scikit-build-core | Configured in `pyproject.toml` |
| C++ tests | doctest | Single header in `third_party/` |
| Python tests | pytest | |
| RL | gymnasium, stable-baselines3, torch (CPU) | |
| Visualization | matplotlib + pillow (GIF), Unity | No ffmpeg required |

### 3.2 Compiler flags

| Flag | Reason |
|---|---|
| `/O2` | Release optimization |
| `/W4 /permissive-` | Warnings and standards conformance |
| `/fp:precise` | **Required for determinism.** Never use `/fp:fast` |
| `/EHsc` | Standard C++ exception model |
| `/utf-8` | Treat source files as UTF-8 |

The Python module is always built as **Release** or **RelWithDebInfo**. A Debug build requires `python3xx_d.lib`, which is not part of a standard Python install.

### 3.3 Units and coordinates

- All units are SI: meters, seconds, radians.
- World frame: origin at the bottom-left corner, x to the right, y up.
- Angles are wrapped to `[-π, π)` by a single `wrap_angle()` function.
- θ = 0 means the robot faces +x; positive ω is counter-clockwise.

### 3.4 Default parameters (initial values)

| Parameter | Value | Description |
|---|---|---|
| Map size | 10 × 10 m | |
| dt | 0.1 s | Fixed time step |
| Robot radius | 0.2 m | Circular body |
| v range | [0, 1.0] m/s | No reversing initially |
| ω range | [-1.5, 1.5] rad/s | |
| LiDAR beams | 36 | 10° apart, relative to heading |
| LiDAR range | 5.0 m | |
| Goal radius | 0.3 m | Success when the robot center is within this distance |
| Max steps | 500 | Then `truncated` |
| Obstacles | 8–12 rectangles | Fixed map in Phase 1 |

All parameters live in a single C++ `Config` struct with a matching Python dataclass.

### 3.5 Observation and action

**Action** (`Box(-1, 1, shape=(2,))`), scaled in Python:
- `a[0]` → v: `(a[0] + 1) / 2 * v_max`
- `a[1]` → ω: `a[1] * ω_max`

**Observation** (`Box`, float32, shape=(40,)):

| Index | Content | Normalization | Range |
|---|---|---|---|
| 0–35 | LiDAR distances | `d / range` | [0, 1] |
| 36 | Distance to goal | `d / map diagonal` | [0, 1] |
| 37 | sin of goal bearing | relative to heading | [-1, 1] |
| 38 | cos of goal bearing | relative to heading | [-1, 1] |
| 39 | Previous v | `v / v_max` | [0, 1] |

The bearing is encoded as sin/cos to avoid the discontinuity at ±π.

### 3.6 Reward (v1)

```
r = k_progress * (d_prev - d_now)
  - c_step
  + R_goal        (if the goal is reached)
  - R_collision   (if a collision occurs)
```

Initial values: `k_progress = 1.0`, `c_step = 0.01`, `R_goal = 10`, `R_collision = 10`.
Every reward change is recorded in `docs/DECISIONS.md` together with its training result.

### 3.7 Termination

| Event | `terminated` | `truncated` | `info` |
|---|---|---|---|
| Goal reached | True | False | `is_success=True` |
| Collision | True | False | `collision=True` |
| Max steps | False | True | `TimeLimit.truncated=True` |

**Collision definition:** the distance from the robot center to the closest point of an obstacle (or boundary wall) is `<= robot radius`. Tangential contact counts as a collision.

### 3.8 Randomness and determinism

- `std::uniform_real_distribution` and similar are **not used**: their output can differ between standard library implementations.
- Each environment owns its RNG (e.g. PCG32 or splitmix64 + xoshiro) with a hand-written `uniform(a, b)`.
- No mutable state is shared between environments, so thread scheduling cannot affect results.
- Same binary + same seed + same action sequence → bit-identical output.

### 3.9 Code and Git conventions

- All code, comments, commit messages and documentation are in English.
- Formatting: `.clang-format` (ships with VS); `ruff` for Python.
- Commit messages: `phase1: add slab raycast test`.
- Training outputs (`runs/`, model files) are not committed; only result tables and plots are.

---

## 4. Repository layout

```
robotsim/
├── CMakeLists.txt
├── pyproject.toml
├── README.md
├── .gitignore
├── .gitattributes
├── .clang-format
├── cpp/
│   ├── include/robotsim/
│   │   ├── config.hpp
│   │   ├── geometry.hpp      # vectors, AABB, circle, wrap_angle
│   │   ├── rng.hpp           # portable RNG
│   │   ├── world.hpp         # single environment
│   │   ├── vec_world.hpp     # Phase 4
│   │   └── thread_pool.hpp   # Phase 4
│   ├── src/
│   ├── bindings/
│   │   └── module.cpp        # pybind11
│   └── tests/
├── third_party/
│   └── doctest.h
├── python/robotsim/
│   ├── __init__.py
│   ├── config.py
│   ├── env.py                # RobotNavEnv (gymnasium.Env)
│   ├── vec_env.py            # Phase 4, SB3 VecEnv wrapper
│   ├── baselines.py          # random and rule-based policies
│   └── render.py             # matplotlib
├── tests/                    # pytest
├── scripts/
│   ├── train.py
│   ├── evaluate.py
│   ├── benchmark.py
│   └── record_episode.py
├── unity/                    # Phase 6
└── docs/
    ├── PROJECT_PLAN.md
    ├── DECISIONS.md
    └── results/
```

---

## Phase 0 — Setup and skeleton

**Duration:** 1 day
**Goal:** Get the build chain right from the start so later phases do not fight infrastructure.

### 0.1 Tasks

- [ ] Git repository and directory layout
- [ ] Virtual environment with Python 3.12 (`py -3.12 -m venv .venv`)
- [ ] Python packages: pybind11, scikit-build-core, cmake, numpy, pytest; torch (CPU wheel), gymnasium, stable-baselines3, tensorboard, matplotlib, pillow
- [ ] `CMakeLists.txt`: `robotsim_core` static library, `_robotsim` pybind11 module, doctest target behind `BUILD_TESTING`
- [ ] `pyproject.toml` using the scikit-build-core backend
- [ ] MSVC flags from section 3.2
- [ ] Example function: `int add(int, int)` in C++, exposed as `robotsim.add`
- [ ] One doctest and one pytest example
- [ ] Optional: GitHub Actions build and test on `windows-latest`

### 0.2 Commands (x64 MSVC environment)

```powershell
pip install -e . --no-build-isolation

cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure

pytest -q
```

### 0.3 Success criteria

- [ ] `pip install -e . --no-build-isolation` succeeds in a clean venv
- [ ] `python -c "import robotsim; print(robotsim.add(2, 3))"` prints `5`
- [ ] `ctest -C Release` and `pytest` both pass
- [ ] A change to the C++ code is visible in Python after rebuilding
- [ ] The build produces zero warnings at `/W4`

---

## Phase 1 — C++ simulation core

**Duration:** 2–3 days
**Goal:** A fast, headless 2D world whose correctness is proven by tests.

### 1.1 Interface sketch

```cpp
namespace robotsim {

struct StepResult {
    float reward;
    bool terminated;
    bool truncated;
    bool is_success;
    bool collision;
};

class World {
public:
    explicit World(const Config& cfg);

    void reset(uint64_t seed);
    StepResult step(float v, float omega);   // physical units

    // Writes the observation into a caller-provided buffer (zero-copy in Phase 4)
    void write_observation(float* out) const;

    const RobotState& robot() const;
    const std::vector<AABB>& obstacles() const;
    Vec2 goal() const;
};

}
```

### 1.2 Tasks

- [ ] `geometry`: `Vec2`, `AABB`, `wrap_angle`, circle–AABB closest point, ray–AABB intersection (slab method)
- [ ] `rng`: portable RNG and `uniform(a, b)`
- [ ] `World`: fixed map, boundary walls as AABBs
- [ ] Kinematics: `x += v cosθ dt`, `y += v sinθ dt`, `θ += ω dt`, then wrap
- [ ] Velocity clamping
- [ ] Collision checking (section 3.7)
- [ ] Raycast LiDAR: slab test per beam against all obstacles, smallest positive t, clamped to range
- [ ] `reset(seed)`: rejection sampling so the robot and goal are at least `r + margin` from obstacles and at least 2 m apart
- [ ] Reward v1 and termination logic
- [ ] Micro-benchmark for `step` + `write_observation`

### 1.3 Tests

| Test | Check |
|---|---|
| Straight line | θ=0, v=1, 10 steps → x = 1.0 m |
| Constant turn | v=0, ω=π/2, 10 steps → θ = π/2 |
| Circular path | constant v, ω → trajectory lies on the analytic circle |
| Angle wrap | θ = π + 0.1 → -π + 0.1 |
| Ray–AABB | axis-aligned, diagonal, inside-start, miss, corner hit |
| LiDAR on known map | all 36 values on a hand-computed single-obstacle map |
| Collision | intersecting, exactly tangent, 1e-4 apart |
| Valid resets | no start or goal inside an obstacle over 10,000 seeds |
| Determinism | same seed + same random action sequence (1000 steps) → identical under `memcmp` |
| Termination | robot placed on goal → `is_success`; driven into wall → `collision`; 500 steps → `truncated` |

### 1.4 Success criteria

- [ ] Kinematics tests match analytic solutions within **1e-5** (float32) or 1e-9 (double)
- [ ] All raycast and collision tests pass, including edge cases
- [ ] All 10,000 resets are valid
- [ ] Determinism test passes bit for bit
- [ ] Single thread, Release: **≥ 100,000 steps/s** (step + observation, 36 beams, ~12 obstacles)
- [ ] Measured speed and hardware recorded in `docs/results/phase1_speed.md`

---

## Phase 2 — Gymnasium interface

**Duration:** 1–2 days
**Goal:** Use the C++ core as a standard RL environment.

### 2.1 Tasks

- [ ] pybind11 bindings for `World`, `Config`, `StepResult`; observation as `py::array_t<float>`
- [ ] `config.py`: dataclass converting to the C++ `Config`
- [ ] `RobotNavEnv(gymnasium.Env)`
  - [ ] `observation_space`, `action_space` (section 3.5)
  - [ ] `reset(seed=None, options=None)` following Gymnasium seeding rules; C++ seed derived from `self.np_random`
  - [ ] `step(action)`: scaling, C++ call, returns `(obs, reward, terminated, truncated, info)`
  - [ ] `info`: `is_success`, `collision`
  - [ ] `render()` with `render_mode="rgb_array"` (matplotlib top-down view)
- [ ] `gymnasium.register(id="RobotNav-v0", ...)`
- [ ] Python-side speed measurement

### 2.2 Tests (pytest)

- `check_env` emits no warnings (warnings treated as errors)
- 100,000 random steps: no NaN, observations stay within `observation_space`
- Two episodes with the same seed are identical
- `reset(seed=None)` produces different episodes
- Action scaling: `[-1,-1]` → `(0, -ω_max)`, `[1,1]` → `(v_max, ω_max)`

### 2.3 Success criteria

- [ ] `gymnasium.utils.env_checker.check_env(env)` passes **without warnings**
- [ ] 100,000 random steps complete without crashes, NaN or out-of-bounds observations
- [ ] Single-environment steps/s from Python measured and compared to raw C++ speed (wrapper overhead documented)
- [ ] Seed reproducibility verified from Python
- [ ] A rendered frame saved under `docs/results/`

**Expectation:** single-environment speed from Python will be far below raw C++ speed (binding call, NumPy allocation, Python logic). This is normal and motivates Phase 4.

---

## Phase 3 — First training run (MVP)

**Duration:** 2–3 days
**Goal:** Prove that the environment is learnable.

### 3.1 Tasks

- [ ] `baselines.py`
  - [ ] Random policy
  - [ ] Rule-based policy: turn toward the goal; turn toward the open side if front LiDAR beams are below a threshold
- [ ] `train.py`
  - [ ] SB3 PPO, `MlpPolicy`, `DummyVecEnv` with 8 environments
  - [ ] `VecNormalize` decision (reward normalization optional) recorded in DECISIONS
  - [ ] TensorBoard logging, periodic evaluation via `EvalCallback`
  - [ ] Fixed and logged `torch.set_num_threads()`
  - [ ] Training seed, config and git commit hash written to the output folder
- [ ] `evaluate.py`: 1000 episodes on a seed range not used in training; success, collision, timeout rate, mean episode length
- [ ] Repeat with 3 training seeds
- [ ] README v1

Windows note: all executable code in training scripts must be inside `if __name__ == "__main__":`. Multiprocessing on Windows uses `spawn`, and without this guard the script re-executes itself.

### 3.2 Signals to watch while tuning

| Symptom | Likely cause |
|---|---|
| Robot does not move | Collision penalty too large relative to progress reward |
| Robot circles the goal | Goal radius too small, or progress reward rewards oscillation |
| Success stuck near 0% | Broken observation scaling or goal expressed in the wrong frame |
| Evaluation much worse than training | Seed or configuration mismatch |

### 3.3 Success criteria

- [ ] Fixed map, random start/goal, 1000 evaluation episodes: **success ≥ 80%, collision ≤ 10%**
- [ ] Mean ± standard deviation reported over 3 training seeds
- [ ] Comparison table against random and rule-based policies
- [ ] PPO beats the rule-based controller on success or collision rate; if not, the result and interpretation are in the README
- [ ] Wall-clock training time, step count and hardware in the README
- [ ] Learning curve plot (3 seeds, shaded range) in the README
- [ ] Static image of an evaluation episode in the README

---

## Phase 4 — Native vectorization and benchmarks

**Duration:** 3–4 days
**Goal:** Demonstrate performance optimization with measurable data.

### 4.1 Design

```
Python: vec_env.step_async(actions [N,2]) → step_wait()
          │  (single pybind11 call, GIL released)
          ▼
C++ VecWorld
  ├─ obs_buf      [N, 40] float32   (preallocated, stable address)
  ├─ reward_buf   [N]     float32
  ├─ term_buf     [N]     uint8
  ├─ trunc_buf    [N]     uint8
  └─ ThreadPool: environments split into T chunks, one chunk per thread
          │
          ▼
Python: np.ndarray views (zero-copy)
```

### 4.2 Tasks

- [ ] `ThreadPool` based on `std::thread`; threads created once
  - OpenMP is not used: MSVC's default `/openmp` support is limited to OpenMP 2.0
- [ ] `VecWorld(N, cfg, n_threads)`: N `World` instances, shared buffers
- [ ] `step(actions)`: each environment writes only its own row (chunked dispatch to reduce false sharing)
- [ ] Auto-reset: finished environments are reset in the same step; the final observation is copied to `final_obs_buf`
- [ ] pybind11: `py::gil_scoped_release` during `step`
- [ ] Zero-copy: buffers returned as `py::array_t` with `VecWorld` as the owning base object
- [ ] `vec_env.py`: SB3 `VecEnv` subclass
  - `terminal_observation` and `TimeLimit.truncated` in `infos`
  - `dones = terminated | truncated`
- [ ] `benchmark.py`: methods × N ∈ {1, 4, 8, 16, 64}, warm-up then measure, 5 repetitions, median
- [ ] Hotspot analysis with the Visual Studio Performance Profiler

### 4.3 Caveat on zero-copy buffers

The same buffer is overwritten every step. Python code that stores observations by reference will see corrupted values. SB3's rollout buffer copies values, so it is safe, but:
- `vec_env.py` exposes a `copy_obs` option (default: off)
- Copying and zero-copy speeds are benchmarked separately

### 4.4 Benchmark matrix

| Method | N=1 | N=4 | N=8 | N=16 | N=64 |
|---|---|---|---|---|---|
| Single env, Python loop | | – | – | – | – |
| `DummyVecEnv` | | | | | |
| `SubprocVecEnv` | | | | | – |
| Native, 1 thread | | | | | |
| Native, T threads | | | | | |

**Windows note:** `SubprocVecEnv` uses `spawn`, so every worker reloads Python and all modules. Startup time is **excluded**; step throughput is measured after warm-up, and this is stated in the README.

Measurement conditions:
- "High performance" power plan; laptop plugged in
- No heavy background applications
- CPU model, physical/logical core count and RAM listed with the table

### 4.5 Success criteria

- [ ] **Correctness:** every environment in the native VecEnv produces the same trajectory as a single `World` with the same seed, including auto-reset (tested)
- [ ] Results do not change with thread count (determinism test for T=1 and T=8)
- [ ] At N=8, native is **≥ 5× faster** than `SubprocVecEnv` (initial target)
- [ ] Thread scaling plot and parallel efficiency (`speed_T / (T × speed_1)`) in the README
- [ ] **End-to-end impact:** total wall-clock time of the same PPO run with `DummyVecEnv` vs native; environment throughput and overall training throughput reported separately
- [ ] Profiler hotspots identified; at least one optimization attempted and its effect measured
- [ ] `python scripts/benchmark.py` reproduces the table with a single command

**Expectation:** large environment speedups may not translate proportionally into training time, because network updates on CPU PyTorch can become the bottleneck. This is reported and explained, not hidden.

---

## Phase 5 — Realism

**Duration:** 3–5 days
**Goal:** Increase realism step by step, measuring the cost and benefit of each addition.

### 5.1 Features (in order, each toggled via config)

| # | Feature | Parameters |
|---|---|---|
| R1 | LiDAR noise | Gaussian σ (m), beam dropout probability (dropout → max range) |
| R2 | Action noise and slip | Multiplicative/additive noise on v, ω |
| R3 | Domain randomization | v_max, ω_max, σ, dropout sampled per episode from ranges |
| R4 | Procedural maps | Random obstacle count, size and placement per episode; goal reachability guaranteed |
| R5 | Moving obstacles | Circular obstacles with constant speed and direction changes; ray–circle and circle–circle tests |

R4 reachability: BFS on an occupancy grid from start to goal; regenerate the map if no path exists.

### 5.2 Experiments

**Ablation:** each feature added individually and cumulatively:

| Configuration | Success | Collision | steps/s (native, N=64) |
|---|---|---|---|
| Baseline (Phase 3) | | | |
| + R1 | | | |
| + R1 + R2 | | | |
| + ... | | | |

**Robustness:** train two policies:
- A: clean environment
- B: R1 + R2 + R3 enabled

Evaluate both on a series of perturbed environments (increasing noise, different velocity limits). Result: success rate vs. perturbation level plot.

**Generalization:** a policy trained with R4 is evaluated on maps generated from a seed range never used during training.

### 5.3 Success criteria

- [ ] Unit test for each feature (e.g. noisy LiDAR with σ=0 equals clean LiDAR; moving obstacle collision)
- [ ] With all features disabled, behavior is identical to Phase 3 (regression test)
- [ ] Ablation table completed
- [ ] **Robustness:** at the highest perturbation level, B's drop in success rate is clearly smaller than A's (with plot)
- [ ] **Generalization:** success **≥ 70%** on unseen procedural maps (initial target)
- [ ] Collision rate reported separately for the moving obstacle scenario
- [ ] Throughput loss with all features enabled documented as a percentage
- [ ] Determinism preserved with all features enabled

---

## Phase 6 — Visualization

**Duration:** 2–3 days
**Goal:** Make results visible and demonstrate the engine-agnostic architecture.

### 6.1 Episode recording format (JSON)

```json
{
  "version": 1,
  "config": { "map_size": [10, 10], "robot_radius": 0.2, "dt": 0.1 },
  "obstacles": [ { "min": [1, 2], "max": [2, 4] } ],
  "goal": [8.5, 7.0],
  "frames": [
    { "t": 0, "pose": [1.0, 1.0, 0.0], "lidar": [ ... ], "movers": [[x, y]] }
  ],
  "outcome": "success"
}
```

### 6.2 Tasks

- [ ] `record_episode.py`: record an episode with a trained policy
- [ ] Matplotlib GIF: robot, heading arrow, LiDAR beams, goal, trail (`PillowWriter`)
- [ ] Unity replay scene
  - [ ] Load recordings with `JsonUtility` (serializable class definitions)
  - [ ] Floor, walls and obstacles as cubes; robot as a cylinder; LiDAR via `LineRenderer`
  - [ ] Play/pause and speed control
  - [ ] **Coordinate transform:** simulation (x, y) plane → Unity (x, z) plane; the angle sign flips because Unity uses a left-handed frame
- [ ] (Stretch) Run the C++ core live in Unity as a native plugin

### 6.3 Stretch goal: Unity native plugin (Windows)

- C API layer with `extern "C"` and `__declspec(dllexport)`: `rs_create`, `rs_reset`, `rs_step`, `rs_get_pose`, `rs_destroy`
- Separate CMake target: `robotsim_unity` (SHARED)
- DLL placed in `Assets/Plugins/x86_64/`
- C#: `[DllImport("robotsim_unity", CallingConvention = CallingConvention.Cdecl)]`
- **The Unity Editor locks a loaded DLL.** Close the editor before replacing it
- Running the policy inside Unity would require ONNX export and Unity Sentis; the stretch goal is limited to replaying a **recorded action sequence** through the DLL and reproducing the same trajectory

### 6.4 Success criteria

- [ ] GIF of the trained agent at the top of the README (≤ 5 MB)
- [ ] Example GIFs for success, collision and moving obstacles under `docs/results/`
- [ ] The same recording shows the same trajectory in matplotlib and Unity (side-by-side screenshot)
- [ ] Short screen capture or image of the Unity scene in the README
- [ ] (Stretch) Action sequence replayed through the DLL matches the Python trajectory within 1e-5

---

## Phase 7 — Documentation

**Duration:** 1 day
**Goal:** Make the project understandable in two minutes.

### 7.1 README structure

1. GIF + one-sentence description
2. Key results (success/collision, speedup, robustness)
3. Architecture diagram (C++ core → pybind11 → Gymnasium/VecEnv → SB3; C API → Unity)
4. Installation (Windows + MSVC)
5. Usage: train, evaluate, benchmark, record (one command each)
6. Results: Phase 3, 4 and 5 tables and plots
7. Design decisions
8. Limitations and future work (UE5 frontend, 3D, camera observations)

### 7.2 Design decision topics

- Why lock-step with fixed dt; how determinism is ensured (RNG, `/fp:precise`, no shared state)
- Why a custom core instead of an existing bridge such as ML-Agents
- Why a custom thread pool instead of OpenMP
- Risks of zero-copy buffers and how they are handled
- Engine-agnostic design and the planned UE5 frontend

### 7.3 Success criteria

- [ ] Someone unfamiliar with the project can run it on a clean Windows machine by following the README
- [ ] All tables and plots regenerated from the final code
- [ ] `v1.0` tag created

---

## 13. Risks and mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| pybind11 / scikit-build-core setup issues on Windows | Medium | Phase 0 slips | Build the module with plain CMake first, add packaging afterwards |
| PPO does not reach 80% success | Medium | Phase 3 slips | Simplify the map, use a curriculum; continue Phase 4 in parallel |
| Reward hacking (standing still, spinning) | High | Misleading results | Track episode length and timeout rate during evaluation |
| Native speedup does not carry over to training | High | Weaker result | Report environment and training throughput separately, explain the bottleneck |
| CPU training too slow | Medium | Schedule slips | Small network, gradually increase step budget, overnight runs |
| Data races / loss of determinism in threads | Medium | Wrong results | No-shared-state rule + T=1 vs T=8 comparison test |
| Unity plugin integration takes too long | High | Phase 6 slips | JSON replay first; plugin is a stretch goal |
| Large downloads | Medium | Setup delays | CPU-only torch, `--no-build-isolation`, packages installed once |

---

## 14. Windows-specific pitfalls

- Build in an **x64** MSVC environment; the default "Developer PowerShell" shortcut may open an x86 environment.
- Python and MSVC must both target x64.
- Multi-config generator: use `--config Release` and `ctest -C Release`.
- Do not build the Python module in Debug; `python3xx_d.lib` will not be found.
- The compiled module is a `.pyd` tied to a specific Python version.
- A loaded `.pyd` is locked; close Python/Jupyter sessions before rebuilding.
- Scripts using multiprocessing need `if __name__ == "__main__":`.
- `SubprocVecEnv` startup is slow on Windows; measure after warm-up.
- If you hit long-path errors, keep the repository in a short path.
- Never use `/fp:fast`; it breaks determinism.
- Windows PowerShell 5.1 writes UTF-8 **with BOM** via `Set-Content -Encoding utf8`; write config files without BOM.
- Unity locks loaded DLLs until the editor is closed.

---

## 15. Progress tracking

| Phase | Status | Start | End | Tag | Notes |
|---|---|---|---|---|---|
| 0 | ✅ Done | 2026-09-17 | 2026-09-17 | v0.1-phase0 | Smart App Control disabled (D-001) |
| 1 | ✅ Done | 2026-09-17 | 2026-09-18 | v0.2-phase1 | 518,803 steps/s single core (5.2x target) |
| 2 | ✅ Done | 2026-09-18 | 2026-09-18 | v0.3-phase2 | check_env clean; 133,959 steps/s through the Gymnasium API |
| 3 | ⬜ Not started | | | | |
| 4 | ⬜ Not started | | | | |
| 5 | ⬜ Not started | | | | |
| 6 | ⬜ Not started | | | | |
| 7 | ⬜ Not started | | | | |

Status: ⬜ Not started · 🟨 In progress · ✅ Done · ⛔ Blocked
