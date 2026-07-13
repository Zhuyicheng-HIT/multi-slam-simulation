# Simulation Sensor Topic Contract

All estimator-facing sensor messages carry a nonzero `header.stamp` and a nonempty sensor-frame `frame_id`. Algorithms subscribe to the normalized `/sensors/*` topics so normal, injected, and replayed data use the same interface.

| Modality | Simulator/source topic | Estimator-facing topic | Type | Expected frame |
|---|---|---|---|---|
| LiDAR | `/sim/mid360/points_raw` | `/sensors/lidar/points` | `sensor_msgs/PointCloud2` | `mid360_link` |
| FCU IMU | `/mavros/imu/data_raw` via `/livox/imu` | `/sensors/imu` | `sensor_msgs/Imu` | `base_link` |
| GNSS/BDS-compatible fix | `/uav/global_fix` | `/sensors/gnss/fix` | `sensor_msgs/NavSatFix` | antenna/body frame supplied by MAVROS bridge |
| Optical flow | `/sim/optical_flow/rad` | `/sensors/optical_flow/rad` | `mavros_msgs/OpticalFlowRad` | downward optical camera frame |
| RGB-D color | `/front/d435i/color/image_raw` | `/sensors/rgbd/color` | `sensor_msgs/Image` | D435i color optical frame |
| RGB-D aligned depth | `/front/d435i/aligned_depth_to_color/image_raw` | `/sensors/rgbd/depth` | `sensor_msgs/Image` | D435i color optical frame |

`/sensor_contract/diagnostics` reports count, rate, stamp regression/duplication, zero stamps, empty frames, and staleness for every normalized stream. `/fault/state` labels active injected faults and is recorded with every experiment.

## LiDAR Body Exclusion

`pointcloud_body_filter` removes returns inside a configurable axis-aligned volume in `mid360_link`. The initial simulation bounds are `x,y=[-0.45,0.45] m` and `z=[-0.35,0.15] m`. These are conservative configuration values, not calibrated hardware geometry. `/sensors/lidar/body_removed_ratio` must be checked in each world before accepting them.

## Fault Injection

Each modality has a dedicated `fault_injector` instance. Its output topic does not change when a fault is enabled.

| Modality | Supported first-stage faults |
|---|---|
| LiDAR | outage, time offset, random point dropout |
| IMU | outage, time offset, gyro/accel bias, angular saturation |
| GNSS | outage, time offset, ENU-like north/east jump, covariance scaling |
| Optical flow | outage, time offset, low quality, scale error |
| Depth | outage, time offset, random holes |
| Color | outage, time offset, low-texture flattening |

Fault windows use node elapsed time. `fault_duration_s <= 0` means the fault remains active after `fault_start_s`. Random faults use the configured seed.

## Ground-Truth Isolation

`/sim/mid360/ground_truth_odom` and `/sim/mid360/cloud_registered` are evaluator-only. They are intentionally absent from the normalized sensor pipeline and must not be added as estimator inputs. The existing `/uav/local_pose` and `/uav/local_odom` are FCU-fused comparison signals, not optical-flow or LIO corrections.

## rosbag2

Use `scripts/record_sensor_bag.sh` after the simulation, LIO, and sensor pipeline are running. SQLite3 is the baseline format. Replay with `scripts/replay_sensor_bag.sh`, and launch algorithm nodes with `use_sim_time:=true` when consuming `/clock`.
