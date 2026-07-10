# Running the Simulation

Build first and source the workspace in every terminal:

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Terminal 1: Simulator and Sensor Stack

GPS mode with the diagnostic optical-flow pipeline:

```bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_flow.sh
```

Non-GPS mode with optical flow and range injected into ArduPilot:

```bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_nongps_flow.sh
```

The first terminal owns Gazebo, ArduPilot SITL, MAVROS and sensors. Do not run
the complete stack a second time.

## Terminal 2: Flight State Machine

Automatically accept local pose plus GPS or fresh optical flow:

```bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh
```

Force one navigation readiness source:

```bash
NAVIGATION_SOURCE=gps \
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh

NAVIGATION_SOURCE=optical_flow FLOW_MIN_QUALITY=0 \
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh
```

`PREFLIGHT_WAIT_S` is the maximum readiness timeout, not a fixed delay.

## Terminal 3: FAST-LIO Mapping

```bash
LIDAR_WS="$HOME/multi-slam-deps/mid360_ws" RVIZ=1 \
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_mid360_fastlio_mapping.sh
```

Main outputs:

```text
/cloud_registered
/Odometry
/fastlio_denoised_map
/fastlio_occupancy_grid
```

## D435i Visualization

Open separate viewers and select the listed topic in each drop-down. RQt may
remember the previously selected topic.

```bash
ros2 run rqt_image_view rqt_image_view
# /front/d435i/color/image_raw

ros2 run rqt_image_view rqt_image_view
# /front/d435i/depth/image_rect_raw
```

D435i-compatible simulation topics:

```text
/front/d435i/color/image_raw                  sensor_msgs/msg/Image rgb8
/front/d435i/color/camera_info                sensor_msgs/msg/CameraInfo
/front/d435i/depth/image_rect_raw             sensor_msgs/msg/Image 16UC1 mm
/front/d435i/depth/camera_info                sensor_msgs/msg/CameraInfo
/front/d435i/aligned_depth_to_color/image_raw sensor_msgs/msg/Image
/front/d435i/depth/color/points               sensor_msgs/msg/PointCloud2
/front/d435i/gyro/sample                      sensor_msgs/msg/Imu
/front/d435i/accel/sample                     sensor_msgs/msg/Imu
/front/d435i/imu                              sensor_msgs/msg/Imu
```

Sensor data uses best-effort, keep-last depth 1 and volatile durability.

## Diagnostics

```bash
ros2 topic info -v /front/d435i/depth/image_rect_raw
ros2 topic hz /front/d435i/color/image_raw
ros2 topic hz /sim/mid360/points_raw
ros2 run tf2_ros tf2_echo base_link front_d435i_color_optical_frame
ros2 topic echo --once /mavros/state
```

Runtime logs are written under `<workspace>/logs` and intentionally ignored by
Git.

