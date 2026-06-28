# Repository Guidelines

## Overview
`teleop_vr_recv` is the VR teleoperation receiver and coordinate-frame alignment frontend for a ROS 2 workspace. `README.md` is the source of truth for the intended architecture (relative ΔT mapping, smoothing, and workspace guarding before IK).

## Project Structure & Module Organization
- `README.md`: design/architecture spec and constraints.
- Planned module layout (keep these responsibilities separated):
  - `net/`: UDP receive/deserialize, timestamps.
  - `frame/`: VR → robot frame alignment/calibration.
  - `mapping/`: relative ΔT mapping + scaling.
  - `filter/`: pose smoothing (position low-pass, quaternion SLERP).
  - `safety/`: workspace limiter/clamping.
  - `model/`: state/command types (the external interface should be `TeleopCommand`).
- When adding ROS 2 code, prefer `include/teleop_vr_recv/<module>/` for headers, `src/<module>/` for implementations, `test/` for unit tests, and `launch/` for launch files.

## Build, Test, and Development Commands
This repository currently contains documentation only; once `package.xml`/`CMakeLists.txt` are added:
- Build (from workspace root): `colcon build --packages-select teleop_vr_recv`
- Test: `colcon test --packages-select teleop_vr_recv && colcon test-result --verbose`
- Environment: `source install/setup.bash`

## Coding Style & Naming Conventions
- C++: target C++17 (matches other packages in `teleop_ws`), 4-space indentation, no tabs.
- Naming: `snake_case` files (`udp_receiver.cpp`, `frame_transformer.hpp`), `CamelCase` types (`VrRawState`).
- Make frame semantics explicit in identifiers (`vr_*`, `base_*`, `ee_*`) and avoid “absolute pose” control APIs.

## Testing Guidelines
- Add focused unit tests per layer (ΔT math, smoothing, workspace clamp). Name files `test_<module>.cpp`.
- Treat UDP input as untrusted: validate payload size/version, reject NaNs/Infs, and bound rates/values before publishing targets.

## Commit & Pull Request Guidelines
- Git history is minimal; use short, imperative summaries (e.g., `Add UDP receiver`).
- PRs: describe intent, link issues, include a “Test Plan”, and update `README.md` if dataflow or constraints change.

## Agent-Specific Notes (Optional)
- Prefer `rg` for searches, keep diffs minimal, and avoid network access unless explicitly approved.
