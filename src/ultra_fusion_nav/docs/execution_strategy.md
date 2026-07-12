# Execution and Release Strategy

## 1. Iteration Contract

Every behavior-changing iteration follows this loop:

```text
hypothesis
  -> save clean pre-change baseline JSON and logs
  -> modify one primary variable
  -> rebuild affected packages
  -> run the complete rectangle route
  -> report trajectory, yaw/gyro coupling, cloud jump, and timestamp metrics
  -> retain with evidence or revert only that iteration
```

Diagnostic-only code changes still require a pre/post smoke test, but they are not allowed to silently change estimator inputs, parameters, timestamps, frames, or ground-truth access.

## 2. Milestones and Gates

### M0 - Audit and Reproducibility

Status: in progress.

Completed:

- Identified the publication and runtime workspaces.
- Verified Git push authentication and clean `main`.
- Audited ROS/Gazebo/toolchain dependencies.
- Read and visually checked the local Ultra-Fusion paper methodology pages.
- Ran and saved a 125 s clean-main baseline.

Exit gate:

- Clean external FAST-LIO import/build uses the documented default path.
- Three unchanged runs establish metric variance or a documented smaller repeatability set is justified.
- The yaw/gyro correlation failure, registered timestamp duplicates, and cloud-centroid jumps are localized to either estimator, bridge, or metric.
- A single command can build and run the baseline without personal absolute paths.

### M1 - Sensor Data and rosbag2

One primary variable at a time:

1. Define topic/frame/timestamp contracts and `uf_interfaces`.
2. Add a frame/time validator that never consumes ground truth.
3. Add parameterized MID360 body/self-return cropping and verify retained point ratio.
4. Add SQLite rosbag2 recording profiles for normal GPS and non-GPS flow modes.
5. Add deterministic replay launch and topic rate/stamp/QoS checks.
6. Add fault injection by wrapper topics: offset, drift, noise, outage, jump, dropout, depth holes, and low-texture image mode.

Exit gate: a recorded bag replays twice with identical message counts, header-stamp sequences, fault labels, and evaluator results within declared numeric tolerance.

### M2 - LiDAR-IMU Baseline and Diagnostics

1. Freeze the FAST-LIO source/patch set.
2. Publish `/lio/odom`, `/lio/path`, `/lio/local_map`, and `/lidar/points_deskewed` through a stable adapter namespace.
3. Expose match count, point-plane residual statistics, Hessian eigenvalues, spatial support, and map quality without using ground truth.
4. Add TUM export and ATE/RPE evaluation.

Exit gate: stable mapping in at least simple, urban, tunnel, and warehouse scenes; no timestamp regression; diagnostics explain known degeneracy scenes.

### M3 - Independent Reliability Scores

Implement and validate one modality per iteration:

- `D_L`: matches, residuals, Hessian conditioning, support distribution, and dynamic ratio.
- `D_B`: fix metadata, satellites/DOP, covariance, innovation, outage, and jump.
- `D_I`: dynamics, saturation, jerk, timing, and later preintegration residual.
- `D_OF`: quality, range, texture, body-rate compensation evidence, and velocity consistency.
- `D_V`: depth validity, blur, projection consistency, and segmentation confidence.

Exit gate: each injected fault moves its intended evidence and score in the expected direction without relying on evaluator truth at runtime. Ground truth may label plots offline.

### M4 - BDS/GNSS and Optical-Flow Assistance

1. Implement LLA/ECEF/ENU and synthetic integrity metadata.
2. Implement outage/jump detection and smooth re-anchor.
3. Keep raw optical flow distinct from FCU-fused local position.
4. Validate non-GPS horizontal assistance under `D_OF` control.

Exit gate: a 1/3/5 m GNSS recovery jump does not hard-pull the output trajectory, and low-texture flow is disabled or down-weighted before it destabilizes motion.

### M5 - Static Map Protection

1. Add LiDAR temporal-consistency dynamic classification.
2. Publish static, dynamic, and uncertain clouds.
3. Gate map insertion and down-weight uncertain points.
4. Compute adjacent-frame and keyframe feature repeatability with a no-filter ablation.

Exit gate: dynamic objects do not persist in the static map and repeatability improvement is demonstrated without excessive static-point loss.

### M6 - ReliabilityScheduler

1. Implement continuous weights, binary gates, covariance inflation, hysteresis, and reason codes.
2. Implement `NORMAL`, `DEGRADED`, `RISK`, `RELOCALIZING`, `RECOVERED`, and `FAILSAFE` transitions.
3. Replay all single and concurrent degradation bags.

Exit gate: state/weight timelines match a table-driven transition specification and remain stable near thresholds.

### M7 - Offline Unified Sliding Window

1. Select GTSAM or Ceres using the dependency spike.
2. Implement state `{R, p, v, b_a, b_g}`, prior, IMU preintegration, and marginalization.
3. Add GNSS and optical-flow factors.
4. Add a transitional relative LIO pose factor, clearly labeled loose/mid coupling.
5. Inject scheduler weights and run fixed-weight versus dynamic-weight ablations.
6. Add point-to-plane LiDAR factors only after per-point timing, local planes, and correlation handling are valid.

RGB-D masks initially control observation/map admission; they are not called a pose factor unless a geometric residual such as depth consistency or reprojection is actually implemented.

Exit gate: offline optimization is deterministic, bounded, and dynamic weighting improves a predeclared metric across the degradation matrix rather than one selected run.

### M8 - Relocalization and Evaluation Closure

1. Build a static keyframe/relocalization map.
2. Retrieve candidates with Scan Context or an equivalent descriptor.
3. Verify with NDT/ICP and cross-check with available height, heading, and integrity evidence.
4. Smoothly reconnect the recovered pose and protect the map during recovery.
5. Automate normal, outage, jump, LiDAR degeneracy, dynamic target, low texture, and concurrent degradation experiments.

Exit gate: report success rate, recovery time, wrong-relocalization rate, ATE/RPE, availability, and failure cases with reproducible commands.

## 3. First Three Iterations

The current baseline failed despite low trajectory error. Therefore the first implementation sequence is:

1. **M0.1 measurement-only:** extend baseline capture to save raw time-delta distributions and event windows for FAST-LIO yaw, FCU gyro, raw cloud, and registered cloud. No estimator parameter changes.
2. **M0.2 dependency-only:** clean-import the pinned FAST-LIO/Livox workspace at the documented default path and compare outputs to the existing local binary.
3. **M1.0 data-contract-only:** add topic/frame/stamp inventory plus a record/replay smoke profile. No reliability score or backend yet.

Only one of these is active in a branch at a time.

## 4. Commit, Push, and Version Policy

- Make local commits at meaningful single-variable checkpoints.
- Do not push every diagnostic commit.
- Push after a milestone gate passes, with its report and reproducible command.
- Proposed milestone tags:

```text
uf-nav-v0.1-baseline
uf-nav-v0.2-reliability
uf-nav-v0.3-scheduler-map
uf-nav-v0.4-backend
uf-nav-v0.5-evaluation
```

- M0 documentation may be committed locally now, but the first remote push should include the M0 reproducibility fixes and passing repository/build checks rather than an empty package scaffold.
- Before each push: run `python3 tools/verify_repository.py`, inspect `git status --short`, review `git diff --check`, build affected packages, and attach the latest evaluation summary.

## 5. Decision Rules

- Better RViz appearance is not evidence.
- A better ATE with worse timestamps, coupling, or map continuity is a mixed result that requires explanation.
- If a metric is invalid, fix and version the metric before tuning against it.
- If a fault injector uses ground truth to create a fault, the estimator still cannot subscribe to ground truth.
- Synthetic BDS metadata validates logic only; reports must not describe it as receiver-level BDS performance.
- When a stage fails its gate, keep the last passing milestone runnable and continue on a focused diagnostic branch.
