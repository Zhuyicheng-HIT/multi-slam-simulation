# Stable Indoor Baseline 2026-08-24

This baseline is based on `checkpoint/stable-microlink-20260822` and contains
startup fixes only. The fusion algorithm and sensor weighting configuration
were not changed.

## Validation

Validated with the ordinary indoor world:

```bash
LOG_DIR="$PWD/logs/stable_baseline_indoor_aug22_rerun_20260824" \
VALIDATION_WORLD_PATH="$PWD/src/multi_slam_uav_sim/worlds/low_indoor_apm_rgbd_mid360.sdf" \
VALIDATION_WORLD_NAME=low_indoor_apm_rgbd_mid360 \
VALIDATION_GAZEBO_WORLD_NAME=low_indoor_apm_rgbd_mid360 \
VALIDATION_ROUTE=rectangle \
VALIDATION_ROUTE_FEEDBACK_SOURCE=fcu_local \
VALIDATION_LOCALIZATION_SAFETY_ENABLED=true \
VALIDATION_ENABLE_EXTERNALNAV_EKF3=0 \
VALIDATION_ENABLE_VISION=1 \
VALIDATION_RECORD_REPLAY_BAG=false \
VALIDATION_RECORD_RAW_LIDAR=false \
VALIDATION_REQUIRE_FASTLIO_DRIFT=false \
VALIDATION_STOP_OBSERVERS_ON_LANDING=true \
VALIDATION_MINIMUM_SIM_DURATION=0 \
RECTANGLE_LENGTH_X=2 RECTANGLE_LENGTH_Y=1.2 \
RECTANGLE_SPEED=0.2 RECTANGLE_HOLD_TIME=2 \
bash tools/run_unified_rectangle_validation.sh
```

The vehicle completed takeoff, all four rectangle edges, landing, and
disarm. The strict validation result was `passed=true`.

## Accuracy

Metrics are causal and use the frozen initial alignment:

| Metric | Result |
| --- | ---: |
| 3D RMSE | 0.0245 m |
| 3D P95 | 0.0346 m |
| 3D maximum | 0.0523 m |
| XY RMSE | 0.0147 m |
| Z RMSE | 0.0196 m |
| Endpoint error | 0.0211 m |

The run used the unified five-source backend with ExternalNav FCU consumption
disabled for estimator-only evaluation. LiDAR, GNSS, optical flow, RGB-D, and
IMU factor paths were all active; native factor queue loss and optimization
rollback counts were zero.

## Startup fixes

- Wait for MAVROS IMU and simulated barometer before waiting for the LiDAR
  bridge, avoiding a bridge initialization deadlock.
- Load `mavros_apm_rgbd.yaml` so MAVROS connects to the SITL TCP endpoint.
- Use the dependency workspace `local_setup.bash` to preserve the ArduPilot
  Gazebo system plugin path.

Build and test verification after the change:

```text
colcon build --symlink-install: passed
colcon test-result --verbose: 76 tests, 0 errors, 0 failures
```
