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
