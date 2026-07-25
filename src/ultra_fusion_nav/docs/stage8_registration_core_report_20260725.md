# Stage 8 Registration Core Report

Date: 2026-07-25

## Boundary

Ultra-Fusion's public repository currently distributes ROS 2 Humble binaries
and configuration, not the full estimator source, and does not expose a
source-backed Scan Context/NDT/ICP relocalization implementation. This project
therefore uses the installed PCL 1.12 registration library as a separately
owned relocalization verification core.

This milestone does not publish poses, reset ExternalNav, consume Gazebo truth,
or claim online relocalization. It only verifies geometric registration once a
candidate keyframe and an initial transform are available.

## Package

`uf_relocalization` is an `ament_cmake` package containing:

- a bounded ICP wrapper;
- a bounded NDT wrapper;
- explicit source-to-target initial and final transforms;
- convergence, fitness, method, and point-count results;
- input validation for empty clouds and invalid limits.

Both methods use PCL's maintained registration implementations rather than a
project-owned optimizer.

## Verification

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select uf_relocalization --symlink-install
source install/setup.bash
colcon test --packages-select uf_relocalization
colcon test-result --verbose
```

Three GTest cases pass:

1. ICP recovers a known 3D rigid transform from an identity initial guess.
2. NDT refines a deliberately perturbed but reasonable candidate transform.
3. Empty registration input is rejected.

The test executable completed in approximately `3.2 s` on the current WSL
environment.

## Remaining work

1. Define static-keyframe admission using map quality, repeatability, pose
   spacing, and scheduler health.
2. Add a place-recognition interface; Scan Context is preferred but is not yet
   implemented or imported.
3. Verify retrieved candidates with NDT/ICP and reject poor fitness or large
   cross-sensor inconsistency.
4. Publish a relocalization result only after recovery tests measure success
   rate, recovery time, and wrong-relocalization rate.
5. Carry a reset counter through the ExternalNav/MAVLink boundary before any
   FCU-facing recovery experiment.

