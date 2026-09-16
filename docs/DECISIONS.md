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