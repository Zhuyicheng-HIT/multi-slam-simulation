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

## Example

```bash
source /opt/ros/humble/setup.bash
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
ros2 run mid360_sim_bridge_cpp gz_livox_bridge_node --ros-args \
  -p gz_topic:=/mid360/lidar \
  -p livox_lidar_topic:=/livox/lidar \
  -p input_imu_topic:=/mavros/imu/data_raw \
  -p livox_imu_topic:=/livox/imu
```

`/sim/mid360/ground_truth_odom` is published only for simulation evaluation.
It is never connected to FAST-LIO or the unified estimator.
