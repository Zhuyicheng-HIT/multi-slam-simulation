# MID360 Gazebo Direct Livox Bridge

`gz_livox_bridge_node` is a simulation-only adapter. It subscribes to the
Gazebo Transport `gz.msgs.LaserScan` published by the MID360 model and
publishes `livox_ros_driver2/msg/CustomMsg` on `/livox/lidar`. It also copies
the FCU raw IMU from `/mavros/imu/data_raw` to `/livox/imu` while preserving a
monotonic timestamp sequence.

## Data boundary

The production sensor path remains:

```text
MID-360S -> official livox_ros_driver2 -> /livox/lidar, /livox/imu -> FAST-LIO
```

The adapter is only selected by `MID360_SIM_BRIDGE_MODE=direct_livox` for
Gazebo. It does not open a Livox network device and must not be launched with
the real sensor driver. Both paths intentionally provide the same downstream
ROS message types and topic names.

Before publishing, the adapter transforms each return from `mid360_link` into
the aircraft body frame and removes returns inside the configured aircraft
exclusion box. The default rotation is the simulated 15 degree nose-down
mount, and the default body-frame bounds are `x,y=[-0.45,0.45] m` and
`z=[-0.35,0.15] m`. The per-frame removed fraction is published on
`/sensors/lidar/body_removed_ratio`. This filtering happens before FAST-LIO,
so self returns cannot enter scan matching or the map.

## Example

```bash
source /opt/ros/humble/setup.bash
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
ros2 run mid360_sim_bridge_cpp gz_livox_bridge_node --ros-args \
  -p gz_topic:=/mid360/lidar \
  -p livox_lidar_topic:=/livox/lidar \
  -p input_imu_topic:=/mavros/imu/data_raw \
  -p livox_imu_topic:=/livox/imu \
  -p body_filter_enabled:=true
```

`/sim/mid360/ground_truth_odom` is published only for simulation evaluation.
It is never connected to FAST-LIO or the unified estimator.

Gazebo publishes each `LaserScan` as one synchronous snapshot. In the Livox
packet contract the adapter represents that snapshot at the packet end: the
header/timebase is `t_snapshot - scan_period` and every point has
`offset_time=scan_period`. FAST-LIO therefore receives a valid scan interval,
but every point uses the same end pose and no fictitious rolling-scan deskew is
introduced. `synthetic_scan_timing:=true` remains available only for explicit
timing stress tests. The real MID-360S path keeps the official driver's native
per-point offsets.

Timestamp experiments are explicit opt-ins. `stamp_lidar_from_latest_imu:=true`
assigns each snapshot the latest preserved FCU IMU measurement stamp, while
`preserve_sim_scan_clock:=true` epoch-aligns the raw Gazebo clock. Neither is a
general replacement for wall restamping: ArduPilot SITL and Gazebo do not
necessarily advance at the same rate, and the FCU callback can lag a scan.
