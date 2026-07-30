## Draft / WIP

This is a Draft baseline for review. It is intentionally not ready to merge,
and later completed/validated stages will be added to the same branch.

## Problem

The repository had a simulated D435i path, but no clean upstream-ready baseline
that combined paired RGB-D transport, an explicit RTAB-Map profile, a
reproducible D435i-only lifecycle, and quantitative validation tools. This PR
ports only the approved stable visual-SLAM work onto the latest `main`.

## System

```text
Gazebo D435i
  -> C++ RGB-D bridge (Python fallback)
  -> paired RGB + aligned 16UC1 depth + CameraInfo + optical TF
  -> exact-sync RTAB-Map feature_aligned
  -> odometry/map + read-only database and trajectory diagnostics
```

RTAB-Map is evaluation-only and does not feed ArduPilot EKF or the flight
controller.

## Changes

- **Bridge:** C++ RGB-D transport, rate probes, paired shared timestamps,
  CameraInfo, optical frames, QoS and subscriber-aware PointCloud2.
- **RTAB-Map:** `feature_aligned` default, detector/feature type 6,
  `Mem/UseOdomFeatures=true`, exact sync, `base_link` and threshold guards.
- **Workflow:** D435i-only headless simulation, textured world, explicit GUI /
  flow / MID360 switches, one `/clock` publisher and recorded process groups.
- **Validation:** throughput, latency, ATE/RPE, robustness, read-only database,
  A-G route, feature alignment and speed envelope tools.
- **Docs:** focused README, status and benchmark documents plus ignore rules.

See `PR_FILE_LIST.txt` for the exact list.

## One-command launch

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select d435i_rgbd_bridge_cpp multi_slam_uav_sim
source install/setup.bash
RTABMAP_PROFILE=feature_aligned D435I_WORLD=textured \
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_visual_slam_headless.sh
```

## Current validation

Submission-clone checks:

- `git diff --check`: PASS
- Bash syntax: PASS (9 scripts)
- Python syntax: PASS (11 modules)
- YAML/XML parse: PASS
- `colcon build`: PASS for both changed packages
- `colcon test`: PASS; no package tests are currently registered
- privacy/artifact/large-file audit: PASS

The approved stable-run evidence reports:

- 640×480 RGB-D about 28–29 Hz;
- RTAB-Map about 16 Hz;
- feature-aligned visual-word and GlobalClosure chain verified;
- 0.35 m/s recommended test speed;
- 0.75 m/s straight line 3/3 valid PASS;
- no lost, reset, TF-backward or wrong-loop event in valid formal samples;
- Gazebo currently uses `kms_swrast`.

A runtime smoke and full-simulation launch were not repeated in this clone
because the isolated development task owned an active long-route
Gazebo/SITL/MAVROS/RTAB-Map run. No process or ROS topic from that task was
touched.

## Compatibility

The original full simulation keeps its default GUI, D435i, MID360, optical-flow
and MAVROS behavior. The latest upstream 100 Hz MAVROS IMU request is preserved.
This PR does not modify FAST-LIO, MID360 mapper or Ultra-Fusion code.

## Not complete

- real D435i / USB / real-flight validation;
- hardware rendering permission work;
- multi-source fusion and flight-controller feedback;
- any unfinished stage A/B/C development-worktree changes;
- a submission-clone runtime rerun after the active isolated simulation ends.

This PR contains neither real-hardware validation nor multi-source fusion.
Future stable work will be appended as new commits to
`feat/d435i-rgbd-visual-slam` and this same Draft PR. It must not be merged or
marked ready for review yet.
