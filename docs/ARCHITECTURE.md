# 系统架构

## 1. 飞控状态链路

```text
Gazebo 传感器 -> ArduPilot SITL -> MAVROS -> /uav/* -> 伴随计算机节点
```

上层导航节点不得使用 Gazebo 真值位姿替代飞控状态。Gazebo 位姿仅供名称中明确标注为仿真诊断的节点使用。

矩形飞行状态机采用事件驱动的起飞准备：MAVROS 和本地位姿就绪后，GPS 或新鲜光流任一满足要求即可进入稳定确认。`PREFLIGHT_WAIT_S` 是最长超时，不是固定等待时间。

## 2. 伴随计算机传感器

MID360 与前视 D435i 视为直接连接伴随计算机的传感器：

```text
Gazebo MID360 -> gz_mid360_pointcloud_bridge -> /sim/mid360/points_raw
Gazebo D435i  -> d435i_sim_bridge            -> /front/d435i/*
```

下视相机模拟类似 MTF-01P 的光流输入。光流诊断和向飞控注入光流是两种独立启动模式：默认模式便于对比仿真运动，非 GPS 模式才把光流与距离数据送入 ArduPilot。

## 3. D435i 接口

仿真适配器发布常见 `realsense2_camera` 风格接口，包括：

- 彩色图与相机内参；
- 16UC1 毫米深度图与对齐深度；
- 彩色光学坐标系中的 `PointCloud2`；
- 加速度、角速度、组合 IMU 与静态 TF。

仿真 RGB 与深度成像器共址，因此对齐深度有效，但不模拟真实 D435i 的双目基线。Gazebo 模型也没有两只物理分离的红外相机，所以不发布虚假的红外双目话题。

## 4. TF 关系

```text
base_link
  |-- front_d435i_link
  |     |-- front_d435i_color_frame
  |     |     `-- front_d435i_color_optical_frame
  |     `-- front_d435i_depth_frame
  |           `-- front_d435i_depth_optical_frame
  `-- mid360_link
```

光学坐标系遵循 ROS 约定：`+Z` 向前、`+X` 向右、`+Y` 向下。

## 5. FAST-LIO 数据流

```text
/sim/mid360/points_raw
  -> Livox/点云适配
  -> FAST-LIO
  -> /cloud_registered、/Odometry、轨迹
  -> mid360_reliable_mapper
  -> 可靠点云与二维栅格地图
```

FAST-LIO 与 Livox 驱动源码放在独立外部工作空间，本仓库只包含项目自研且体积较小的可靠建图节点。通过 `LIDAR_WS` 指向外部工作空间，避免绑定某个用户目录。

## 6. 路径与资源查找

- 已安装资源通过 ROS 2 package share 查找。
- Shell 脚本通过自身位置推导工作空间前缀。
- ArduPilot、插件、FAST-LIO 和可选场景用环境变量指定。
- Gazebo 资源路径由 `scripts/env.sh` 统一组装。

不得在源码、launch 文件或参数文件中写入个人主目录绝对路径。

## 7. 统一运行时与硬件适配边界

PR24 将仿真和硬件输入收敛到同一条 Livox/传感器契约。仿真时，
`mid360_sim_bridge_cpp` 从 Gazebo 的 `/mid360/lidar`、`/mid360/imu` 读取，
发布硬件兼容的 `/livox/lidar`、`/livox/imu`；真实 MID360 则直接使用官方
`livox_ros_driver2`，不启动仿真 bridge。两条路径都必须保持各自一个发布者，避免重复数据进入 FAST-LIO；启动脚本会在 FAST-LIO 启动前检查并持续监测该所有权。

```text
/livox/lidar -> livox_custom_to_pointcloud -> /sensors/lidar/points_raw
             -> pointcloud_body_filter  -> /sensors/lidar/points
/livox/imu   -> sensor_relay_manager     -> /sensors/imu
```

`uf_sensor_pipeline` 还统一 GNSS、光流和 RGB-D 的话题、时间戳、frame 与 QoS。
真实 MID360 的 Livox IMU 线加速度通常以 `g` 表示，使用
`config/real_mid360_imu_units.yaml` 将其转换为后端所需的 SI 单位；仿真桥已经
直接输出 `m/s^2`。同一观测只能进入统一滑窗一次，故 normalized `/sensors/*`
话题是 Ultra-Fusion 后端的唯一归一化输入边界；FAST-LIO 仍直接消费硬件兼容的
`/livox/*` 输入并输出原生点面信息。

统一后端由 `tools/run_unified_backend_stack.sh` 的 `RUNTIME_PROFILE` 选择模态：
`minimal_lidar_imu`、`four_source`、`five_source` 和 `robustness`（测试故障注入）。
该选择只改变启用的模态和诊断，不改变上游消息契约。
