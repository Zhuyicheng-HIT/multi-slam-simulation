# MID360 可靠建图与 Nano 飞行部署

这是 **Nano 平台真实飞行边缘部署包**。它只包含真实 MID360 雷达运行需要的内容：Livox 驱动、FAST-LIO2、可靠点云建图、二维占据栅格、MID360 模块相对无人机机体的安装外参 TF，以及无头运行脚本。

本包不包含 Gazebo、模型、world、仿真 launch、RViz 默认启动流程。真实飞行时建议无头运行，RViz 放在地面站或调试电脑上远程查看。

## 1. 包结构

```text
mid360_reliable_mapper_nano_flight_xxx/
  nano_flight/
    edge_env.sh
    scripts/
      00_create_venv.sh
      01_install_dependencies.sh
      02_build.sh
      03_edit_mount_config.sh
      04_check_lidar_network.sh
      05_run_flight_mapper.sh
      06_record_safety_bag.sh
      07_status_topics.sh
      08_stop_mapper.sh
  src/
    mid360_reliable_mapper/
    FAST_LIO_ROS2/
    livox_ros_driver2_real/
```

## 2. 独立虚拟环境

本包使用独立文件夹作为 ROS workspace，并提供本地 Python venv：

```bash
.venv/
build/
install/
log/
```

不会 source 你 Nano 上其它工程。所有脚本都会通过：

```bash
source nano_flight/edge_env.sh
```

只激活当前文件夹环境。

## 3. 导入 Nano

把压缩包拷到当前目录后：

```bash
mkdir -p ~/mid360_flight_ws
cd ~/mid360_flight_ws
tar -xzf ../mid360_reliable_mapper_nano_flight_xxx.tar.gz --strip-components=1
```

如果是 zip：

```bash
mkdir -p ~/mid360_flight_import
unzip ../mid360_reliable_mapper_nano_flight_xxx.zip -d ~/mid360_flight_import
mkdir -p ~/mid360_flight_ws
cp -a ~/mid360_flight_import/mid360_reliable_mapper_nano_flight_xxx/. ~/mid360_flight_ws/
```

后续默认都以 `~/mid360_flight_ws` 为例。

## 4. 依赖安装

你已经有 Ubuntu 22.04。如果已经有 ROS 2 Humble：

```bash
cd ~/mid360_flight_ws
bash nano_flight/scripts/01_install_dependencies.sh
```

如果还没有 ROS 2 Humble，并且 Nano 能联网：

```bash
cd ~/mid360_flight_ws
INSTALL_ROS_HUMBLE=1 bash nano_flight/scripts/01_install_dependencies.sh
```

如果还没有 Livox SDK2，并且 Nano 能联网：

```bash
cd ~/mid360_flight_ws
INSTALL_LIVOX_SDK2=1 bash nano_flight/scripts/01_install_dependencies.sh
```

创建独立 venv：

```bash
bash nano_flight/scripts/00_create_venv.sh
```

## 5. 编译

```bash
cd ~/mid360_flight_ws
bash nano_flight/scripts/02_build.sh
```

内存紧张时：

```bash
BUILD_JOBS=2 bash nano_flight/scripts/02_build.sh
```

## 6. 工作区隔离检查

Nano 上如果以前装过其它工作区，例如：

```text
~/fast_lio2_ws
~/ws_livox
~/ws_air_competition
```

里面可能也有 `livox_ros_driver2` 和 `fast_lio`。运行本包前不要 source 这些旧工作区。建议每次新开终端只执行：

```bash
source /opt/ros/humble/setup.bash
source ~/mid360_flight_ws/nano_flight/edge_env.sh
```

然后检查三个包必须来自当前工作区：

```bash
bash ~/mid360_flight_ws/nano_flight/scripts/09_verify_workspace_isolation.sh
```

期望输出路径都在当前工作区：

```text
$MID360_WS/install
```

如果 `ros2 pkg prefix livox_ros_driver2` 或 `ros2 pkg prefix fast_lio` 指向旧工作区，不要继续运行，重新打开一个干净终端再 source 本包环境。
## 7. MID360 网络

默认 MID360 IP：

```text
192.168.1.123
```

建议 Nano 有线网卡：

```text
IPv4: 192.168.1.50
Mask: 255.255.255.0
```

检查：

```bash
cd ~/mid360_flight_ws
bash nano_flight/scripts/04_check_lidar_network.sh
```


### MID360 UDP 抓包调试

如果 ping 正常但 ROS 没有点云，可以用 `tcpdump` 看 UDP 数据是否进到 Nano。有线网卡名以实际为准，例如 `enP8p1s0`：

```bash
ip -4 addr
sudo timeout 10 tcpdump -i enP8p1s0 -n udp and host 192.168.1.123
```

也可以查看网口状态：

```bash
sudo ethtool enP8p1s0
```

## 8. 安装角配置

你说的安装角是 **整颗 MID360 模块相对无人机机体的安装角**，直接改这个文件：

```bash
nano ~/mid360_flight_ws/src/mid360_reliable_mapper/config/mid360_mount_extrinsic.yaml
```

示例，MID360 整体向下俯仰 12 度：

```yaml
rotation_deg:
  roll: 0.0
  pitch: -12.0
  yaw: 0.0
```

这个配置表示：

```text
base_link -> MID360 模块/body frame
```

FAST-LIO2 发布：

```text
camera_init -> body
```

为了避免 TF 冲突，系统自动取逆发布：

```text
body -> base_link
```

最终链路：

```text
camera_init -> body -> base_link
```

注意：不要用 `LIDAR_TO_IMU_PITCH_DEG` 调整整颗雷达相对无人机机体的安装角。`LIDAR_TO_IMU_*` 是 MID360 内部 LiDAR-IMU 外参调试入口。

## 9. 真实飞行无头运行

正式边缘运行，建议优先使用无头模式：

```bash
cd ~/mid360_flight_ws
bash nano_flight/scripts/05_run_flight_mapper.sh
```

这个命令会启动：

- `livox_ros_driver2_node`
- `fastlio_mapping`
- `mid360_mount_static_tf`
- `fastlio_cloud_mapper_node`
- `pointcloud_occupancy_grid_node`

不会启动 RViz，不会启动仿真。

## 10. 远程桌面实时可视化

如果你通过远程桌面连接 Nano，并且希望实时看到点云和栅格图，使用这个入口：

```bash
cd ~/mid360_flight_ws
bash nano_flight/scripts/05_run_flight_mapper_with_rviz.sh
```

这个命令会在同一个 ROS launch 中启动真实雷达建图和两个 RViz 窗口：

- 三维点云窗口：显示 `/cloud_registered_reliable`、`/fastlio_denoised_map` 和轨迹。
- 二维栅格窗口：显示 `/fastlio_occupancy_free_cells`、`/fastlio_occupancy_occupied_cells` 和轨迹。

注意：RViz 会占用 GPU/CPU。真正飞行时，如果 Nano 负载高，建议只运行无头建图，把 RViz 放在地面站电脑上远程订阅 ROS 话题。

如果远程桌面里打不开 RViz，先确认：

```bash
echo $DISPLAY
rviz2
```

如果 `rviz2` 不存在，重新安装依赖：

```bash
sudo apt install ros-humble-rviz2 mesa-utils
```

## 11. 状态检查

另开一个终端：

```bash
cd ~/mid360_flight_ws
source nano_flight/edge_env.sh
bash nano_flight/scripts/07_status_topics.sh
```

重点检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered_reliable
ros2 topic hz /fastlio_occupancy_grid
ros2 topic echo /Odometry --once
ros2 run tf2_ros tf2_echo body base_link
ros2 run tf2_ros tf2_echo camera_init base_link
```

## 12. 安全录包

飞行前或地面测试建议录包：

```bash
cd ~/mid360_flight_ws
DURATION=60 bash nano_flight/scripts/06_record_safety_bag.sh
```

默认录制：

- `/livox/lidar`
- `/livox/imu`
- `/Odometry`
- `/path`
- `/cloud_registered`
- `/cloud_registered_reliable`
- `/fastlio_denoised_map`
- `/fastlio_occupancy_grid`
- `/tf`
- `/tf_static`

## 13. 停止

```bash
cd ~/mid360_flight_ws
bash nano_flight/scripts/08_stop_mapper.sh
```

## 14. 常见编译问题

如果编译 `livox_ros_driver2` 时报错：

```text
Could not find LIVOX_LIDAR_SDK_LIBRARY
liblivox_lidar_sdk_shared.so
```

说明 Nano 上还没有安装 Livox SDK2。联网时执行：

```bash
cd ~/mid360_flight_ws
INSTALL_LIVOX_SDK2=1 bash nano_flight/scripts/01_install_dependencies.sh
bash nano_flight/scripts/02_build.sh
```

如果 Nano 不能联网，需要先离线安装 Livox SDK2，并确保系统能找到：

```bash
ldconfig -p | grep livox
```
## 15. 飞行前最低检查

真正飞行前至少确认：

1. `ping 192.168.1.123` 正常。
2. `/livox/lidar` 和 `/livox/imu` 频率稳定。
3. `/Odometry` 静止时不明显漂移。
4. `/cloud_registered_reliable` 有输出。
5. `/fastlio_occupancy_grid` 有输出。
6. `tf2_echo body base_link` 能看到配置的安装角。
7. 地面移动测试时，地图不会整体飞走。
8. Nano CPU、内存、温度和供电稳定。

本包提供建图前端，不替代飞控 failsafe、避障策略、地面安全员和实机测试流程。
