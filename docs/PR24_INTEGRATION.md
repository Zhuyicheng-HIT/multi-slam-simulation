# PR #24 集成能力说明

PR #24 将 runtime stack、传感器数据归一化、MID360 兼容适配和在线融合后端合并到主线。本页给出各组件的入口和边界，便于从干净环境复现。

## 传感器栈与归一化

先启动 Gazebo、ArduPilot SITL、MAVROS2 和仿真传感器：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash src/multi_slam_uav_sim/scripts/run_apm_sensor_stack.sh
```

`run_apm_sensor_stack.sh` 默认使用 `MID360_SIM_BRIDGE_MODE=direct_livox`，把 Gazebo 的 MID360 数据转换为硬件一致的 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）和 `/livox/imu`。仿真适配器只用于 Gazebo；真实 MID-360S 必须使用官方 `livox_ros_driver2`，两条路径共用下游接口但不能同时启动。

传感器归一化、故障注入和契约监控由 `uf_sensor_pipeline` 提供：

```bash
ros2 launch uf_sensor_pipeline sensor_pipeline.launch.py \
  enable_lidar:=true enable_gnss:=true \
  active_modalities:="[lidar, imu, gnss, optical_flow]"
```

归一化输出包括 `/sensors/lidar/points`、`/sensors/imu`、`/sensors/gnss/fix_unthrottled` 和 `/sensors/optical_flow/rad`。该层使用消息时间戳和声明的 frame；不要以回调到达时间重新排序或重复包装同一观测。

## MID360 + FAST-LIO

使用硬件兼容的 Livox `CustomMsg` 输入启动 FAST-LIO：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash src/multi_slam_uav_sim/scripts/run_mid360_fastlio_mapping.sh
```

脚本会检查 `/livox/lidar` 和 `/livox/imu` 各只有一个发布者，避免重复桥接导致时间序列交错。需要使用旧的 `PointCloud2` 适配器时显式设置 `FASTLIO_INPUT_MODE=pointcloud START_LIVOX_POINTCLOUD_BRIDGE=1`；默认 `livox` 路径应保持唯一。

## 在线统一后端

在传感器和 FAST-LIO 就绪后启动在线后端：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_unified_backend_stack.sh
```

后端将 FAST-LIO 原生 point-to-plane LiDAR 信息、MID360 IMU、GNSS 和 optical-flow 因子放入同一个有界滑窗。统一输出为 `/fusion/unified/odom`；飞控 ExternalNav 入口和严格验收流程见 [`STABLE_FIVE_SOURCE_EXTERNALNAV_20260816.md`](STABLE_FIVE_SOURCE_EXTERNALNAV_20260816.md)。Gazebo 真值和 MAVROS local pose 只能用于评估或 setpoint 表达，不能回灌估计器。

## 动态观测（默认关闭）

动态 observer 是旁路诊断功能，不改变 `/livox/lidar`、FAST-LIO 或统一后端的数据所有权：

```bash
ros2 launch uf_dynamic_observer observer.launch.py enabled:=true
```

Clean Scan Gateway 仅用于独立 A/B：

```bash
ros2 launch uf_dynamic_observer clean_gateway.launch.py enabled:=true
```

启用 Gateway 前应使用独立命名空间和复现实验；生产 LiDAR 不应直接切换到清洗后的话题。动态 observer 的输入、输出和因果队列约束见 [`uf_dynamic_observer/README.md`](../src/ultra_fusion_nav/uf_dynamic_observer/README.md)。

## 快速验证

```bash
python3 tools/ci_smoke_test.py
python3 tools/test_sensor_pipeline_e2e.py
```

前者检查文本编码、Python/XML/YAML/Shell 语法和纯 Python 协议测试；后者需要已构建 ROS 2 工作区，适合在 Ubuntu 环境中验证传感器话题契约。
