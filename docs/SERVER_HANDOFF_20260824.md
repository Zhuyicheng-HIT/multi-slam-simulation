# Server handoff (2026-08-24)

## Goal and non-negotiable branch separation

This document is written for a fresh server-side Codex with no conversation
history.

- Continue stable-system development from
  `integration/current-complete-pr14-20260824` at
  `054a6744cf2265bd4dc1bd4cee0be6287cd2dbc1` (tag
  `current-complete-pr14-20260824`).
- Continue LiDAR directional experiments only from
  `feat/lidar-directional-reliability-v1` at
  `90e8ff3e429cbf873c94b51ca65cb03d2aacdb0e` (tag
  `lidar-dir-001-directional-reliability-20260824`).
- Never treat the LiDAR experiment as the production baseline. Its current
  decision is **DO_NOT_PROMOTE**, and XYZ/subspace handoff must stay default-off.
- Do not rewrite PR #14 or PR #15, existing branches or frozen tags.

The authoritative repository is
`https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git`.

## Minimal server checkout

Stable-system work:

```bash
git clone https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git
cd multi-slam-simulation
git fetch --all --tags --prune
git switch --detach current-complete-pr14-20260824
git switch -c server/current-complete-work
```

LiDAR experiment work should use a separate worktree:

```bash
git worktree add ../multi-slam-lidar-dir \
  feat/lidar-directional-reliability-v1
```

Read `docs/CURRENT_ALGORITHM_BASELINE_20260824.md`,
`docs/CURRENT_COMPLETE_PR14_SYNC_20260824.md`, and, on the experimental branch,
`docs/LIDAR_DIR_001_DIRECTIONAL_RELIABILITY.md` before running experiments.

## Reproducible environment and initial verification

Use Ubuntu 22.04 and ROS 2 Humble. Follow the repository dependency/setup
scripts rather than vendoring ArduPilot, FAST-LIO or Livox source into Git.
Source external dependency workspaces using `local_setup.bash`; sourcing their
generated `setup.bash` can restore an obsolete project underlay.

Before changing code:

1. record GPU/renderer, ROS distro, compiler, CPU/RAM and dependency commits;
2. build the complete overlay;
3. run all colcon tests and static checks;
4. reproduce the latest-PR14 rectangle profile;
5. verify Raw MID360 ownership, one-observation-one-factor, ExternalNav,
   zero optimization error/rollback/overflow, and sole MAVROS publisher
   `flight_command_arbiter`.

Do not use OpenCV CUDA/OpenCL as evidence that Gazebo has GPU acceleration;
inspect OpenGL/EGL renderer directly.

## Immediate LiDAR task: identical-input full-trajectory A/B/C

Do not tune reliability parameters before this experiment. For each scenario,
generate or record the Raw sensor stream exactly once. Freeze that input and
run three isolated replays with identical initial state, configuration and
input ordering:

- A — scalar production baseline;
- B — XYZ directional handoff;
- C — eigensubspace directional handoff.

First scenario set:

1. normal rich 3-D;
2. 45-degree rotated corridor;
3. partial FoV / sector dropout;
4. long tunnel stress.

The record must contain the causal inputs needed by the complete trajectory
path, including Raw MID360 point timing and IMU. Include GNSS, optical flow and
visual streams when the selected frozen profile requires them. Do not create a
fresh simulation for each method: A/B/C must consume the same record.

### Causality and scoring contract

- Online reliability may use only current/past sensor state.
- Gazebo or motion-capture truth is strictly offline evaluator input.
- Truth must never select a weak direction, gate a factor, tune a threshold or
  enter estimator/reliability topics.
- Preserve NativeLidarFactor ownership, prediction gates, estimator integrity,
  rollback and one-observation-one-factor.
- Do not reduce factor counts or sensor rates to manufacture runtime gains.

### Per-run output

Record, with a shared run identifier and immutable input hash:

- 3-D and per-axis XYZ ATE/RPE;
- weak-direction drift/error and strong-direction error;
- online detected weak direction;
- conditional information eigenvalues/eigenvectors;
- Native factor admission and rejection reason;
- prediction-gate rejection;
- posterior/marginal covariance;
- optimizer error, transaction rollback and queue overflow;
- solver and callback P50/P95/P99;
- backend and total process CPU/RAM.

Report every run, not only the best. Separate physical degeneracy from stream
health failures such as stale, dropout, non-finite or timestamp errors.

## Dataset and rosbag policy

No large rosbag is part of this handoff or ordinary Git history. No existing
local bag is required to reproduce the current frozen claims. The older tunnel
and B3 records predate the latest PR #14 body-filter geometry and must not be
used as the decisive identical-input dataset.

For new server captures, use a non-repository location such as:

```text
/srv/multi-slam/datasets/lidar-dir-001/<scene>/<capture-id>/
```

For every record, create a small manifest containing filename, SHA256, byte
size, source scenario/world, seed, capture command, ROS/Gazebo time contract,
sensor configuration and recommended server path. Commit only the manifest,
never the large bag, database, PCD, build/install/log or Gazebo cache. If a
remote artifact store is used, record its immutable URI and checksum.

## Production invariants during LiDAR work

- `/livox/lidar` remains the production Raw source.
- Dynamic Clean is localization-only; Raw obstacle safety must continue to see
  people, animals and other real obstacles.
- Clean Gateway remains fail-open and default-off as a production replacement.
- XYZ and eigensubspace LiDAR handoff remain default-off outside explicit A/B/C
  experiments.
- Do not change Dynamic ownership, ExternalNav, EKF3, Z-axis policy, safety
  thresholds, command priority or estimator integrity gates.
- Planner and active relocalization intents go through
  `flight_command_arbiter`; no second MAVROS automatic setpoint publisher is
  allowed.
- Z-COV candidates marked DO_NOT_PROMOTE are not part of this handoff.

## Decision gate for future promotion

Do not promote merely because C detects the correct eigendirection. Promotion
requires repeated, identical-input full trajectories showing that the chosen
method improves weak-direction drift without materially degrading total
ATE/RPE, strong-direction information, Z observability, factor availability,
solver integrity, queue behavior or runtime. If A/B/C evidence remains mixed,
keep the feature default-off and report the Pareto boundary.

## Current blockers and recommended sequence

1. Capture latest-PR14 frozen inputs for the four scenario families.
2. Implement/verify deterministic full online-backend replay without changing
   estimator semantics.
3. Run A/B/C on identical inputs, repeated without cherry-picking results.
4. Analyze factor admission, prediction gate and covariance before changing
   parameters.
5. Only after evidence identifies a real causal weakness, make one-variable
   changes on the experimental branch and rerun the full matrix.
6. Re-run stable rectangle, Dynamic fail-open, safety ownership and complete
   build/test before any promotion proposal.

The principal blocker is missing latest-envelope identical-input trajectory
evidence, not missing parameter tuning. Real MID360/UAV validation remains a
separate hardware gate after simulation/replay promotion.
