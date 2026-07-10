# `multi_slam_uav_sim` GitHub 迁移说明

本文说明该包迁移到 GitHub 时的内容边界、路径规则与接口约定。

## 1. 应提交的项目资源

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

这些内容是项目专用且复现必需的源码、参数和小型资源。

## 2. 不应提交的外部依赖

- ArduPilot：<https://github.com/ArduPilot/ardupilot>
- ArduPilot Gazebo 插件与基础 Iris 模型：<https://github.com/ArduPilot/ardupilot_gazebo>
- 通过 ROS 2 软件包安装的 MAVROS；
- Gazebo Sim Harmonic 软件包；
- 通过根目录 `dependencies.repos` 下载的 FAST-LIO 与 Livox ROS Driver 2；
- ArduPilot、插件和外部建图工作空间的编译产物；
- `$HOME/projects/multi-slam-simulation/external` 下的可下载大型场景仓库。

## 3. 路径规则

- 包内脚本从已安装的 package share 目录解析资源。
- `ARDUPILOT_DIR` 默认 `$HOME/ardupilot`。
- `ARDUPILOT_GAZEBO_DIR` 默认 `$HOME/ardupilot_gazebo`。
- `MULTI_SLAM_EXTERNAL_DIR` 默认 `$HOME/projects/multi-slam-simulation/external`。
- `LIDAR_WS` 用于指定外部 FAST-LIO/Livox 工作空间。
- Gazebo 资源路径由 `scripts/env.sh` 统一构造。
- 源码与 launch 文件不得依赖特定用户主目录，也不得直接写死工作空间 `install` 目录；运行时应从脚本自身位置推导。

## 4. 接口规则

- 飞控及其导航传感器通过 MAVROS 与 `/uav/...` 暴露。
- 激光雷达和 RGB-D 是伴随计算机直连传感器，保留在 `/sim/...` 与 `/front/d435i/...`。
- D435i 仿真接口包含 RGB、CameraInfo、16UC1 毫米深度、对齐深度、PointCloud2、加速度、角速度、组合 IMU 和 TF。
- 点云按需以 10 Hz、图像四倍降采样生成，字段为彩色光学坐标系中的 XYZ，不包含颜色纹理。
- 仿真 RGB 与深度相机共址，所以对齐深度有效，但不模拟真实 D435i 双目基线。
- Gazebo 真值话题不得作为算法节点的导航状态。
- 默认光流测试发布 `/sim/optical_flow/raw`，用于诊断，不自动注入飞控。

## 5. 飞行准备规则

矩形飞行由 `guided_rectangle_waypoints` 提供。MAVROS 和本地位姿就绪后，以下任一来源可释放状态机：

- 有效 GPS 定位；
- 质量不低于 `FLOW_MIN_QUALITY` 的新鲜光流。

来源还需要保持 `NAVIGATION_STABLE_S`。`PREFLIGHT_WAIT_S` 是超时，不是固定延迟；`NAVIGATION_SOURCE` 可设置为 `auto`、`gps` 或 `optical_flow`。

完整仓库级打包规则见根目录 [打包原则](../../docs/PACKAGING.md)。
