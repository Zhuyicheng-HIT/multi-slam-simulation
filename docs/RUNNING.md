# 仿真运行与排错

以下命令均使用推荐安装路径，可以原样复制。

## 1. 终端 1：仿真与传感器

GPS 模式，并保留光流诊断：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_sim_with_flow.sh
```

非 GPS 模式，向 ArduPilot 注入光流与距离数据：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_sim_with_nongps_flow.sh
```

终端 1 统一管理 Gazebo、ArduPilot SITL、MAVROS2 和传感器。不要重复启动完整栈，否则会发生端口、进程和话题冲突。

### GPS + 光流融合后输入 ExternalNav

启动机载电脑侧 GPS/光流互补融合，并将唯一融合状态通过
`/mavros/odometry/out` 送入 ArduPilot EKF3：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_sim_with_externalnav.sh
```

该入口默认使用图像 LK 光流、100x100 输入、关闭 D435 点云，并对光流图像采用
best-effort、keep-last 深度 1。桥只保留最新图像并以 15 Hz 发布，旧帧不会排队：

```bash
FLOW_PUBLISH_ALL_FRAMES=false FLOW_BRIDGE_HZ=15.0 \
  bash tools/run_sim_with_externalnav.sh
```

仅排查传感器模型时可设置 `FLOW_PUBLISH_ALL_FRAMES=true`。算法评测必须保持
`FLOW_USE_PHYSICS=false`，否则光流会借助 Gazebo 位姿真值。Gazebo 真值评估器只写
`externalnav_accuracy.json`，不会回灌融合器。性能门限和阶段耗时写入同目录的
`simulation_performance.json`。

ExternalNav 专用入口默认不启动 D435 和 MID360 的 ROS 数据转换桥，以避免在
GPS/光流迭代期间复制无用的 RGB-D 和点云消息。需要同时观察这些传感器时显式恢复：

```bash
ENABLE_D435_BRIDGE=1 ENABLE_MID360_BRIDGE=1 \
  bash tools/run_sim_with_externalnav.sh
```

只记录 GPS/光流导航链时使用轻量 rosbag 档：

```bash
UF_BAG_PROFILE=nav \
  bash src/ultra_fusion_nav/scripts/record_sensor_bag.sh
```

完整多传感器里程碑仍使用 `UF_BAG_PROFILE=full`。

## 2. 终端 2：飞行状态机

自动接受 GPS 或新鲜光流：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_rectangle_state_machine.sh
```

只接受 GPS：

```bash
NAVIGATION_SOURCE=gps \
  bash "$HOME/projects/multi-slam-simulation/tools/run_rectangle_state_machine.sh"
```

只接受光流：

```bash
NAVIGATION_SOURCE=optical_flow FLOW_MIN_QUALITY=0 \
  bash "$HOME/projects/multi-slam-simulation/tools/run_rectangle_state_machine.sh"
```

关键变量：

- `NAVIGATION_SOURCE=auto|gps|optical_flow`：选择起飞准备来源；
- `FLOW_MIN_QUALITY`：允许的最低光流质量；
- `NAVIGATION_STABLE_S`：导航来源需要保持的稳定时间；
- `PREFLIGHT_WAIT_S`：最长准备超时，不是固定延时。

## 3. 终端 3：FAST-LIO 建图

打开 RViz：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_fastlio_mapping.sh
```

无界面建图：

```bash
RVIZ=0 \
  bash "$HOME/projects/multi-slam-simulation/tools/run_fastlio_mapping.sh"
```

确认原始点云和建图输出：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/projects/multi-slam-simulation/install/setup.bash"
ros2 topic hz /sim/mid360/points_raw
ros2 topic hz /cloud_registered
ros2 topic echo --once /Odometry
ros2 topic echo --once /fastlio_occupancy_grid
```

若原始点云正常但没有 `/cloud_registered`，执行：

```bash
test -f "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
ros2 pkg prefix fast_lio
ros2 pkg prefix livox_ros_driver2
```

## 4. D435i RGB-D 可视化

彩色图：

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在下拉框选择 `/front/d435i/color/image_raw`。深度图再打开一个窗口并选择 `/front/d435i/depth/image_rect_raw`。

D435i 仿真话题：

```text
/front/d435i/color/image_raw
/front/d435i/color/camera_info
/front/d435i/depth/image_rect_raw
/front/d435i/aligned_depth_to_color/image_raw
/front/d435i/depth/color/points  # disabled by default; ENABLE_D435_POINTCLOUD=true to enable
/front/d435i/accel/sample
/front/d435i/gyro/sample
/front/d435i/imu
```

传感器数据采用 best-effort、keep-last 深度 1 和 volatile durability。自定义订阅器收不到数据时先检查 QoS。

## 5. 综合诊断

```bash
source /opt/ros/humble/setup.bash
source "$HOME/projects/multi-slam-simulation/install/setup.bash"
ros2 topic echo --once /mavros/state
ros2 topic info -v /front/d435i/depth/image_rect_raw
ros2 topic hz /front/d435i/color/image_raw
ros2 topic hz /sim/mid360/points_raw
ros2 run tf2_ros tf2_echo base_link front_d435i_color_optical_frame
```

运行日志位于 `$HOME/projects/multi-slam-simulation/logs`，该目录被 Git 忽略。

## 6. 漂移、偏航耦合与点云突变检测

先启动仿真、FAST-LIO和矩形飞行，再开一个终端执行：

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/analyze_slam_drift.py --duration 125 \
  --output /tmp/multi_slam_slam_report.json
```

分析器对比 FAST-LIO `/Odometry` 与 Gazebo MID360 真值里程计，检查：

- 位置 RMSE 与最大位置误差；
- 偏航 RMSE 与最大偏航误差；
- FAST-LIO偏航角速度与飞控 HIGHRES_IMU 陀螺 Z 轴的延迟补偿相关系数；
- 原始点云、注册点云和飞控 IMU时间戳回退次数；
- 连续注册点云体素重叠率和质心跳变。

终端输出 `"passed": true` 表示所有阈值通过，完整 JSON 同时写入 `/tmp/multi_slam_slam_report.json`。

FAST-LIO 的主 IMU 固定为飞控 `HIGHRES_IMU -> /mavros/imu/data_raw -> /livox/imu`，不会使用 D435i IMU。默认验收线为位置 RMSE 0.75 m、偏航 RMSE 12 度、稳态及终点偏航误差 15 度、延迟补偿后的 IMU 相关系数 0.65。
