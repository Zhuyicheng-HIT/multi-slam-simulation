# multi_slam_uav_sim GitHub Migration Notes

This package contains the small project-specific assets that should be committed:

- `models/iris_apm_rgbd`
- `models/d435i_downward_sensor_only`
- `models/d435i_downward_rgbd`
- `models/mid360_3d_lidar_sensor_only`
- `models/lidar_downward_sensor_only`
- `models/textured_person`
- `models/textured_vehicle`
- `worlds/apm_city_rgbd_mid360.sdf`
- `worlds/simple_apm_rgbd_mid360.sdf`
- `config/*.yaml`
- `params/*.parm`
- `multi_slam_uav_sim/*.py`
- `scripts/*.sh`

Do not commit these external dependencies. Install or clone them separately:

- ArduPilot: `https://github.com/ArduPilot/ardupilot`
- ArduPilot Gazebo plugin and base Iris models: `https://github.com/ArduPilot/ardupilot_gazebo`
- MAVROS from ROS 2 packages
- Gazebo Sim / Harmonic packages
- FAST-LIO and Livox ROS Driver 2 (download using `dependencies.repos`)
- ArduPilot and `ardupilot_gazebo` source or compiled output
- Large map repositories under `<workspace>/external`

Path policy:

- Package scripts resolve paths relative to the installed package share directory.
- External paths are configured with environment variables:
  - `ARDUPILOT_DIR`, default `$HOME/ardupilot`
  - `ARDUPILOT_GAZEBO_DIR`, default `$HOME/ardupilot_gazebo`
  - `MULTI_SLAM_EXTERNAL_DIR`, default `<workspace>/external`
  - `LIDAR_WS`, default `<workspace>/external/mid360_ws`
- Gazebo resource paths are constructed by `scripts/env.sh`.
- No source or launch file may depend on a specific user's home directory or
  directly source files from `<workspace>/install`; installed scripts locate
  their package share and workspace prefix from their own location.

Interface policy:

- Flight-controller and attached navigation sensors are exposed through MAVROS and `/uav/...`.
- LiDAR and RGB-D are direct companion-computer sensors and stay on `/sim/...` or `/camera/...`.
- The rigid front D435i-style sensor uses `/front/d435i/...`. Its RGB, CameraInfo,
  16UC1 millimetre depth, aligned-depth, PointCloud2, accel, gyro, combined IMU,
  and TF interfaces mirror common `realsense2_camera` names. Infrared stereo is
  intentionally not published because the Gazebo model does not simulate two
  physically separated infrared imagers.
  The point cloud is generated on demand at 10 Hz with 4x image decimation
  (160x120) to keep the Python simulation adapter from throttling camera frames.
  It contains XYZ fields in the color optical frame but is not color-textured.
  Aligned depth is valid because the simulated RGB and depth imagers are
  co-located; it does not model the physical D435i stereo baseline.
- Gazebo ground truth topics must not be used by algorithm nodes as navigation state.
- Optical flow testing publishes `/sim/optical_flow/raw` with `mavros_msgs/msg/OpticalFlow`; it is deliberately not injected into the FCU by default.
- Rectangle flight testing is provided by `guided_rectangle_waypoints`, which uses MAVROS GPS/GUIDED local setpoints while `flow_gazebo_accuracy` compares optical flow with Gazebo motion.
- Rectangle preflight is event driven: after MAVROS and local pose are ready,
  either a valid GPS fix or fresh optical flow above `FLOW_MIN_QUALITY` releases
  the state machine after `NAVIGATION_STABLE_S`. `PREFLIGHT_WAIT_S` is a timeout,
  not a fixed delay. `NAVIGATION_SOURCE` can be `auto`, `gps`, or `optical_flow`;
  the default flow threshold is zero (fresh-message presence), while deployments
  that require stronger flow validation can raise `FLOW_MIN_QUALITY`.
