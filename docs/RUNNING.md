# 仿真运行说明

每个新终端都先进入仓库并加载 ROS 2 与本工作空间：

```bash
cd <仓库目录>
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 1. 终端 1：仿真与传感器

GPS 模式，并保留光流诊断链路：

```bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_flow.sh
```

非 GPS 模式，向 ArduPilot 注入光流与距离数据：

```bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_nongps_flow.sh
```

终端 1 统一管理 Gazebo、ArduPilot SITL、MAVROS 和传感器。不要重复启动完整栈，否则会发生端口、进程和话题冲突。

## 2. 终端 2：飞行状态机

自动接受 GPS 或新鲜光流：

```bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh
```

只接受 GPS：

```bash
NAVIGATION_SOURCE=gps \
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh
```

只接受光流：

```bash
NAVIGATION_SOURCE=optical_flow FLOW_MIN_QUALITY=0 \
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh
```

关键变量：

- `NAVIGATION_SOURCE=auto|gps|optical_flow`：选择起飞准备来源；
- `FLOW_MIN_QUALITY`：允许的最低光流质量，默认 `0` 表示先确认消息新鲜；
- `NAVIGATION_STABLE_S`：来源满足条件后需要保持的稳定时间；
- `PREFLIGHT_WAIT_S`：最长准备超时，不是固定延时。

## 3. 终端 3：FAST-LIO 建图

```bash
LIDAR_WS="$HOME/multi-slam-deps/mid360_ws" RVIZ=1 \
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_fastlio_mapping.sh
```

若不需要打开 RViz：

```bash
LIDAR_WS="$HOME/multi-slam-deps/mid360_ws" RVIZ=0 \
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_fastlio_mapping.sh
```

先确认原始点云，再确认 FAST-LIO 输出：

```bash
ros2 topic hz /sim/mid360/points_raw
ros2 topic list | grep -E 'cloud_registered|Odometry|path|grid'
ros2 topic hz /cloud_registered
ros2 topic echo --once /Odometry
```

若原始点云正常但没有建图输出，检查 `LIDAR_WS` 是否正确、外部工作空间是否已编译并 source、Livox 消息类型是否与 FAST-LIO 参数匹配。

## 4. D435i RGB-D 可视化

分别打开彩色图和深度图窗口，并从下拉框手动选择话题：

```bash
ros2 run rqt_image_view rqt_image_view
# /front/d435i/color/image_raw
```

```bash
ros2 run rqt_image_view rqt_image_view
# /front/d435i/depth/image_rect_raw
```

D435i 风格仿真话题包括：

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

传感器数据采用 best-effort、keep-last 深度 1、volatile durability。若自定义订阅器收不到数据，先检查 QoS 是否兼容。

## 5. 诊断命令

```bash
ros2 topic info -v /front/d435i/depth/image_rect_raw
ros2 topic hz /front/d435i/color/image_raw
ros2 topic hz /sim/mid360/points_raw
ros2 run tf2_ros tf2_echo base_link front_d435i_color_optical_frame
ros2 topic echo --once /mavros/state
```

运行日志写入 `<仓库目录>/logs`，该目录被 Git 忽略，不属于发布内容。
