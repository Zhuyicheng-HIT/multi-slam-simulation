# Ultra-Fusion-Inspired UAV Navigation

This directory contains the project-owned navigation algorithms built around the ideas in Ultra-Fusion. The current scope is Ubuntu 22.04, ROS 2 Humble, Gazebo Sim, ArduPilot SITL, and deterministic rosbag2 replay. Hardware integration is explicitly out of scope until the simulation gates pass.

The implementation order is deliberate:

1. Make every sensor stream recordable, replayable, independently observable, and independently scoreable.
2. Stabilize the LiDAR-IMU baseline and expose the evidence needed by reliability scoring.
3. Add modality scores and the ReliabilityScheduler.
4. Protect the static map and validate BDS/GNSS and optical-flow fallback behavior.
5. Connect only stable observations to an offline unified sliding-window backend.
6. Add relocalization and the full fault-injection evaluation matrix.

The first backend may consume a relative LIO pose as a transitional factor, but it must be described as loose/mid coupling. A tightly coupled Ultra-Fusion-style backend requires point-to-plane LiDAR residuals and must not combine a FAST-LIO pose with a second factor built from the same IMU without accounting for correlation.

## Current Status

| Stage | Status | Gate |
| --- | --- | --- |
| M0 repository/dependencies | accepted for current simple-map scope | clean manifest-pinned FAST-LIO/Livox build |
| M1 sensor data layer | implemented | normalized topics, faults, body crop, rosbag2 record/replay |
| M2 LiDAR-IMU baseline | accepted on fixed simple route | four consecutive passing runs, including one post-scoring change |
| M3 reliability scores | formula semantics accepted; evidence still incomplete by modality | unit tests, ROS endpoint policy, 11-level monotonic sweep, full flight |
| M4 BDS/GNSS and optical flow | integrated, acceptance rerun pending | three GPS/flow ExternalNav rectangle flights passed; corrected flow interval evaluator still needs repeated confirmation |
| M5 dynamic map protection | not started | temporal static/dynamic split and repeatability ablation required |
| M6 ReliabilityScheduler | implementation and ROS runtime sequence pass | live five-modality fault campaign and LIO-backed flow residual still pending |
| M7-M8 backend/relocalization | not started | do not bypass M4-M6 evidence gates |

The next gates are three repeated flights with the corrected optical-flow interval
evaluator, followed by the full LIO and dynamic-point pipeline needed to supply
independent residuals to the scheduler. See `docs/paper_formula_audit.md`,
`docs/score_validation_report.md`, `docs/bds_integrity_report.md`,
`docs/optical_flow_report.md`, and `docs/reliability_scheduler_report.md`.

## Layout

```text
ultra_fusion_nav/
  README.md
  config/                  shared configuration ownership and naming rules
  docs/
    repo_audit.md          M0 environment, repository, and baseline audit
    dependency_plan.md     pinned dependency and package plan
    execution_strategy.md  gates, iteration order, and release policy
  scripts/                 reproducible build, run, bag, and evaluation entrypoints
  tests/                   unit, launch, replay, and regression tests

  # Added only when their stage starts:
  uf_interfaces/           messages and services shared across packages
  uf_sensor_pipeline/      topic normalization, fault injection, and bag profiles
  uf_reliability/          D_L, D_B, D_I, D_OF, and D_V estimators
  uf_aiding/               GNSS outage/jump admission and smooth re-anchor core
  uf_reliability/          modality scores plus factor weights and health-state machine
  uf_backend/              offline sliding-window estimator
  uf_relocalization/       keyframe map and Scan Context/NDT/ICP pipeline
  uf_evaluation/           ATE/RPE, plots, ablations, and scenario matrix
```

The umbrella directory is not a ROS package. Each ROS package will be created at the stage that owns it so early builds do not contain empty packages or speculative interfaces.

## Non-Negotiable Boundaries

- Gazebo ground truth is evaluator-only and is never an estimator input.
- MAVROS local position is a comparison signal, not a correction for FAST-LIO or the future backend.
- `/mavros/imu/data_raw` remains the main LiDAR-IMU input; D435i IMU is a separate visual-inertial experiment input.
- Sensor algorithms use message timestamps and declared frames, not callback arrival time.
- Dynamic or uncertain points cannot enter the static relocalization map without an explicit admission decision.
- Every retained parameter change must beat or explain the saved baseline using the same route and metrics.

See `docs/execution_strategy.md` before starting an implementation iteration.
