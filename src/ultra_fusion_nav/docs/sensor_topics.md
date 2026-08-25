# Simulation Sensor Topic Contract

All estimator-facing sensor messages carry a nonzero `header.stamp` and a nonempty sensor-frame `frame_id`. Algorithms subscribe to the normalized `/sensors/*` topics so normal, injected, and replayed data use the same interface.

Gazebo camera, flow-module IMU, and flow-module pose use simulation source time for one internally consistent integration window. Gazebo first publishes `/sim/optical_flow/rad_native`; the active MTF-01P adapter encodes and decodes the MAVLink(APM) `OPTICAL_FLOW` and `DISTANCE_SENSOR` wire messages before publishing `/sim/optical_flow/rad`. The simulation path is intentionally limited to 15 Hz to control rendering and bridge load. A directly connected MTF-01P keeps its 100 Hz source cadence: the first device timestamp is anchored to ROS time and subsequent device intervals are preserved. Cross-modal algorithms must check timestamp-domain overlap and monotonicity. RGB-D cross-modal time normalization remains an open visual-front-end task.

| Modality | Simulator/source topic | Estimator-facing topic | Type | Expected frame |
|---|---|---|---|---|
| LiDAR | `/sim/mid360/points_raw` | `/sensors/lidar/points` | `sensor_msgs/PointCloud2` | `mid360_link` |
| FCU IMU | `/mavros/imu/data_raw` via `/livox/imu` | `/sensors/imu` | `sensor_msgs/Imu` | `base_link` |
| GNSS/BDS-compatible fix | `/uav/global_fix` | `/sensors/gnss/fix` | `sensor_msgs/NavSatFix` | companion-side default 5 Hz (measured target link), source header stamp preserved; fresh `/sensors/gnss/raw` metadata is paired when available but never blocks a fix |
| Optical flow | `/sim/optical_flow/rad_native` -> MAVLink(APM) -> `/sim/optical_flow/rad` | `/sensors/optical_flow/rad` | `mavros_msgs/OpticalFlowRad` | `mtf01_flow_frd` |
| RGB-D color | `/front/d435i/color/image_raw` | `/sensors/rgbd/color` | `sensor_msgs/Image` | D435i color optical frame |
| RGB-D aligned depth | `/front/d435i/aligned_depth_to_color/image_raw` | `/sensors/rgbd/depth` | `sensor_msgs/Image` | D435i color optical frame |

`/sensor_contract/diagnostics` reports count, rate, stamp regression/duplication, zero stamps, empty frames, and staleness for every normalized stream. `/fault/state` labels active injected faults and is recorded with every experiment.

For a directly connected physical MTF-01P in MAVLink(APM) mode, the corresponding
source topics are `/hardware/mtf01/mavlink_frame`,
`/hardware/mtf01/optical_flow/rad`, and `/hardware/mtf01/range`. The physical
contract is 100 Hz over 115200 baud LVTTL, 42 degree optical-flow FOV, 1.5 degree
ranging FOV, and 0.08 m minimum optical-flow working distance. Range values are
published over the full configurable 0.01--12 m envelope, while the estimator
admits optical-flow factors only over 0.08--12 m. Select the hardware flow topic
through the existing `optical_flow_input_topic` launch argument; do not mix it
with FCU-routed or Gazebo flow in the same run.

The ranging covariance follows the specified 4 cm accuracy from 0.02--2 m and
2% above 2 m. The 7 m/s limit at 1 m is treated as an angular-flow limit whose
planar speed envelope scales with valid range. Illumination above 60 lux is not
observable from MAVLink directly; the reported `quality` field remains the
runtime proxy. The 808 nm laser, 5 V supply, and approximately 100 mA average
current are hardware integration requirements rather than estimator inputs.

The estimator does not solve at 100 Hz. It keeps the source timestamps and
integrates all MTF-01P packets in each roughly 10 Hz LiDAR state interval. This
preserves the real measurement bandwidth without scheduling 100 optimizations
per second. To select the directly connected sensor for a hardware run:

```bash
USE_SIM_TIME=false \
OPTICAL_FLOW_INPUT_TOPIC=/hardware/mtf01/optical_flow/rad \
bash tools/run_unified_backend_stack.sh
```

## LiDAR Body Exclusion

The LiDAR input boundary removes returns inside a configurable axis-aligned
volume after transforming points from `mid360_link` to the aircraft body
frame. The `direct_livox` C++ adapter performs this operation before publishing
`/livox/lidar`; the legacy PointCloud2 path uses `pointcloud_body_filter`.
The initial simulation bounds are `x,y=[-0.45,0.45] m` and
`z=[-0.35,0.15] m`. These are conservative configuration values, not
calibrated hardware geometry. `/sensors/lidar/body_removed_ratio` must be
checked in each world before accepting them.

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

`/sim/mid360/ground_truth_odom` and `/sim/mid360/cloud_registered` are evaluator-only. They are intentionally absent from the normalized sensor pipeline and must not be added as estimator inputs. Gazebo sensor implementations may use simulator state internally to synthesize noisy measurements, but they must not publish that state through the estimator-facing contract. The existing `/uav/local_pose` and `/uav/local_odom` are FCU-fused comparison signals, not optical-flow or LIO corrections.

## rosbag2

Use `scripts/record_sensor_bag.sh` after the simulation, LIO, and sensor pipeline are running. SQLite3 is the baseline format. Replay with `scripts/replay_sensor_bag.sh`, and launch algorithm nodes with `use_sim_time:=true` when consuming `/clock`.
