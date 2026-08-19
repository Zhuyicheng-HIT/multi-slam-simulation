# Dynamic/static map baseline freeze

Freeze date: 2026-08-19 (Asia/Shanghai)

## Immutable source

- Repository: `https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git`
- Pull request: `#14 [codex] freeze low-altitude five-source ExternalNav baseline`
- PR URL: `https://github.com/Zhuyicheng-HIT/multi-slam-simulation/pull/14`
- PR base: `feat/five-source-stage3-integration`
- PR head: `feat/core-algorithm-cleanup-20260817`
- Verified remote head: `50e96f63d19e8d9292b15a684f0cc8a76f55e5bd`
- Commit time: `2026-08-19T13:13:59+08:00`
- Commit subject: `freeze low-altitude five-source ExternalNav baseline`

The GitHub PR metadata and `origin/feat/core-algorithm-cleanup-20260817`
resolved to the same SHA after a pruned fetch.

## Local freeze refs

- Baseline branch: `baseline/pr14-low-altitude-five-source`
- Annotated tag: `baseline-pr14-low-altitude-five-source-20260819`
- Tag object: `3b6a19cfe6e877766671e73d26bf0c315a901f43`
- Tag target: `50e96f63d19e8d9292b15a684f0cc8a76f55e5bd`
- Development branch created from the tag:
  `feat/dynamic-static-map-freedom-v1`

The baseline branch and tag are local only. No remote PR, base branch, or tag was
modified. Development after this point must not occur on the baseline branch.

## Worktree safety audit

Before fetching or creating refs, all discovered local clones were checked with
`git status --short --branch`. No uncommitted user modification was present. No
reset, checkout-overwrite, stash, clean, or deletion was used. Immediately after
the freeze, the development worktree was clean on the exact tag target.

## Reproduction result at the frozen commit

Environment:

- Ubuntu 22.04 under WSL2
- ROS 2 Humble
- existing MID360 dependency overlay:
  `/home/zyc/multi-slam-deps/mid360_ws/install`
- RMW used for the successful build/test path: Fast DDS

Results:

- 16 ROS packages built successfully with `--symlink-install`.
- `colcon test` completed for all 16 packages.
- `colcon test-result --all --verbose`: 75 test records, 0 errors, 0 failures,
  0 skipped. The backend's own aggregated Python runner reported 283 tests OK;
  the visual package reported 13 tests OK.
- No source modification was needed for build or test reproduction.

## Critical low-altitude validation status

The frozen `tools/run_frozen_low_figure8_validation.sh` entry was invoked without
recording a bag. The first attempt exposed the absence of
`rmw_cyclonedds_cpp`; selecting the already installed Fast DDS implementation
resolved that middleware issue. The simulator then stopped before producing
`/livox/lidar` because the nested validation launcher hard-codes
`REQUIRE_GAZEBO_GPU=1` while this WSL session cannot establish the required
render path (`renderD128` is unavailable and the stack falls back to software
rendering). Supplying `REQUIRE_GAZEBO_GPU=0` to the outer wrapper cannot override
the nested hard-coded value.

Therefore:

- Build and unit/integration tests: **PASS**.
- Local flight re-run in this desktop session: **ENVIRONMENT BLOCKED before
  sensor startup**.
- Five-source fusion, ExternalNav, the low-altitude route, and
  one-observation-one-factor were not changed to bypass the block.
- This is not evidence of an estimator or route regression, and it is also not a
  fresh flight PASS. The PR's recorded validation remains the source baseline
  until a compliant GPU/offscreen environment is available.

Follow-up on 2026-08-19: the WSL/WSLg stack was updated and a direct EGL/OpenGL
hardware probe now verifies `D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)`.
Gazebo/Ogre2 hardware rendering, all 14 route checkpoints, LAND, and disarm were
reproduced. The remaining strict-gate failures are independent of the GPU path
and are documented in `docs/WSL2_GAZEBO_GPU_RECOVERY_20260819.md`; the full
frozen acceptance result therefore remains not reproduced.

Ignored diagnostic logs:

- `logs/baseline_freeze_pr14_fastrtps_20260819/sim_launcher.log`
- `logs/baseline_freeze_pr14_cpu_20260819/sim_launcher.log`

## Frozen contracts

Future dynamic-point work must preserve:

1. NativeLidarFactor, IMU, GNSS, optical flow, D_V-weighted vision, and the
   unified backend.
2. ArduPilot ExternalNav and the frozen low-altitude large figure-eight route.
3. Exactly one backend factor for each accepted observation.
4. Original ownership of `/livox/lidar`; an observer may subscribe but must not
   remap or replace the FAST-LIO input during the first phase.
5. No ground-truth signal may enter the detector.
