# 仿真运行与排错

以下命令均使用推荐安装路径，可以原样复制。

## 1. 终端 1：仿真与传感器

GPS 模式，并保留光流诊断：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_flow.sh
```

非 GPS 模式，向 ArduPilot 注入光流与距离数据：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_nongps_flow.sh
```

终端 1 统一管理 Gazebo、ArduPilot SITL、MAVROS2 和传感器。不要重复启动完整栈，否则会发生端口、进程和话题冲突。

## 2. 终端 2：飞行状态机

自动接受 GPS 或新鲜光流：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh
```

只接受 GPS：

```bash
NAVIGATION_SOURCE=gps \
  bash "$HOME/projects/multi-slam-simulation/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh"
```

只接受光流：

```bash
NAVIGATION_SOURCE=optical_flow FLOW_MIN_QUALITY=0 \
  bash "$HOME/projects/multi-slam-simulation/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh"
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
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_mid360_fastlio_mapping.sh
```

无界面建图：

```bash
RVIZ=0 \
  bash "$HOME/projects/multi-slam-simulation/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_mid360_fastlio_mapping.sh"
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
/front/d435i/depth/color/points
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
