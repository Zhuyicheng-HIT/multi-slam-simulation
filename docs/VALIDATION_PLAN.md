# 当前主线验证方案

本文档用于验证 PR #24–#31 合并后的 `main`。验证分为五个层级：静态检查、ROS 2
构建与单元测试、仿真链路、HybridFusion/视觉功能，以及真实 MID360 硬件。每次
修改 runtime、sensor、launch、config 或 CI 后，都应更新本清单对应项目的结果。

## 0. 验证记录

- 验证日期：`YYYY-MM-DD`
- Git commit：``
- ROS 发行版：`Humble / 其他`
- Ubuntu/WSL 版本：``
- Gazebo、ArduPilot、MAVROS 版本：``
- MID360 固件/`livox_ros_driver2` 版本：``
- 运行人员与日志目录：``

结果标记：`[ ] 未执行`、`[x] 通过`、`[!] 失败或需跟进`、`[-] 不适用`。

## 1. 静态与 CI 检查（每次提交必做）

- [ ] GitHub Actions `CI` 成功，且对应 commit 为待发布的 `main`。
- [ ] 本地运行 `python tools/ci_smoke_test.py`。
- [ ] Python 文件全部通过 `py_compile`。
- [ ] YAML/YML、XML、SDF 可以解析。
- [ ] Shell 脚本通过 `bash -n`。
- [ ] 没有 UTF-8 解码错误、乱码标记或冲突标记。
- [ ] `git diff --check` 无空白错误。
- [ ] 仓库没有遗留 build、log、ROS 节点、Gazebo 进程或飞控监听端口。

当前 CI 的边界：它只做上述轻量检查和 4 组基础协议/path 测试，不负责 ROS 2、
Gazebo、C++ 编译、真实传感器或飞控验收。

## 2. ROS 2 构建与单元测试

- [ ] 在干净工作区执行：

  ```bash
  colcon build --symlink-install
  ```

- [ ] 执行完整测试：

  ```bash
  colcon test
  colcon test-result --verbose
  ```

- [ ] 至少单独复测以下受影响包：

  ```bash
  colcon test --packages-select \
    multi_slam_uav_sim \
    mid360_sim_bridge_cpp \
    uf_sensor_pipeline \
    uf_backend_fusion \
    uf_dynamic_observer \
    uf_visual_frontend \
    uf_shared_mapping \
    uf_relocalization \
    hybridfusion_map_fusion
  ```

- [ ] `uf_backend_fusion`：IMU preintegration、anchor snapshot、统一 odometry
  单调时间戳、recent-LiDAR admission、anchor-change retry、scan contract。
- [ ] `uf_sensor_pipeline`：传感器 topic、frame、时间戳、单位和故障注入测试。
- [ ] `mid360_sim_bridge_cpp`：Livox `CustomMsg` 字段、点时间偏移、IMU 转换和
  机身点云过滤测试。
- [ ] `uf_dynamic_observer`、`uf_relocalization`：只在对应功能被启用时执行，
  记录实验性测试是否影响稳定默认路径。
- [ ] C++ 测试没有 error、failure 或 unexpected skip；失败时保存 `log/` 中的
  测试结果，不能只记录“构建成功”。

## 3. 仿真传感器与统一后端链路

- [ ] 使用当前推荐入口启动仿真，不同时启动旧的 MID360 bridge：

  ```bash
  ENABLE_D435_BRIDGE=1 ENABLE_MID360_BRIDGE=1 \
    bash src/multi_slam_uav_sim/scripts/run_apm_sensor_stack.sh
  ```

- [ ] 确认仿真 MID360 使用 `MID360_SIM_BRIDGE_MODE=direct_livox`，输出硬件兼容
  的 `/livox/lidar` 和 `/livox/imu`。
- [ ] 确认 normalized sensor pipeline 输出：
  `/sensors/imu`、`/sensors/rgbd/color`、`/sensors/rgbd/depth`、
  `/sensors/rgbd/camera_info`。
- [ ] 确认 FAST-LIO 消费 MID360 Livox 输入，统一后端消费
  `/fast_lio/native_lidar_factor` 与同源 `/sensors/imu`。
- [ ] 确认统一输出为 `/fusion/unified/odom`，没有第二个节点发布同一权威输出。
- [ ] 记录每个关键 topic 的消息类型、频率、frame_id、时间戳单调性和丢包情况。
- [ ] 检查 IMU 加速度是否使用 SI 单位；仿真与真实设备的转换配置不能混用。
- [ ] 完成一次短矩形或现有稳定路线：启动、校准、运行、降落、disarm 全部完成。
- [ ] 运行结束后确认所有由脚本创建的进程均已退出，日志和结果目录可复现。

## 4. 统一 odometry 专项检查

- [ ] 固定频率模式下只有 publication timer 作为 unified odometry writer。
- [ ] `legacy_hybrid`、`fixed_rate_propagated`、`lidar_event_propagated` 的配置
  行为与文档一致；默认模式不得意外改变。
- [ ] 模拟 optimizer commit 与 IMU propagation 并发，确认 anchor 变化后最多重试一次。
- [ ] 确认 recent-LiDAR 节流、IMU 新鲜度、scan prediction contract、最小发布间隔
  和时间戳单调性保护仍然生效。
- [ ] 检查 diagnostic 中至少包含：`state_trigger_source`、`lidar_factor_source`、
  `output_source`、`live_propagation_reason`、发布/拒绝计数和非单调抑制计数。
- [ ] 运行 5–10 分钟静止 replay，确认无持续增长的 `anchor_changed`、
  `nonmonotonic_output`、`native_worker_errors` 或 queue discard。

## 5. 视觉与 HybridFusion（需要视觉功能时执行）

- [ ] `uf_visual_frontend` 的 RGB-D、CameraInfo、TF、depth encoding 和
  reprojection tests 通过。
- [ ] HybridFusion 默认关闭时不启动 exporter、不广播 TF、不覆盖 FAST-LIO 或
  RTAB-Map 地图。
- [ ] 运行确定性数据集生成器和 evaluator：

  ```bash
  ros2 run hybridfusion_map_fusion run_hybridfusion_benchmark.sh \
    /workspace/logs/hybridfusion/benchmark_validation
  ```

- [ ] 检查 `result.json`、`transform.yaml`、`fused_map.pcd`、`run_manifest.yaml`
  均生成；失败运行也保留结果。
- [ ] 记录 initial/GICP/Hybrid 三种方法的收敛、误差、inlier、voxel growth 和
  runtime；不得把 generated deterministic 数据当作真实硬件精度。
- [ ] 若启用 live exporter，确认 RGB/depth 时间戳严格配对，CameraInfo 与 TF 可用，
  保存服务成功，且源 topic/map 没有被修改。

## 6. 真实 MID360 适配验证（无硬件时标记待执行）

- [ ] 只启动官方 `livox_ros_driver2` 和真实硬件输入管线，不启动 Gazebo bridge。
- [ ] 使用真实设备配置，并明确记录 `imu_acceleration_scale`；确认加速度从 `g`
  转换到后端所需的 SI 单位。
- [ ] 检查 `/livox/lidar`、`/livox/imu` 的消息类型、设备时间、ROS 时间和 frame。
- [ ] 静止 5–10 分钟：检查 IMU 均值、漂移、时间戳回退、队列增长和 odometry 频率。
- [ ] 系留或桨叶锁定状态下完成短距离直线/矩形路线，检查轨迹闭合、偏航、回到
  起点误差以及 unified odometry 连续性。
- [ ] 分别记录 LiDAR–IMU 时间偏移、安装外参、网络丢包和 driver 重连；缺少这些
  记录时，结果不能作为硬件验收证据。
- [ ] 仿真通过不等于真实设备通过；真实设备测试失败时先回滚到传感器适配层，
  不要直接修改后端权重或放宽安全门槛。

## 7. 飞控 ExternalNav 接入前门槛

- [ ] 仿真和硬件 odometry 均已通过时间戳、frame、频率和失效保护检查。
- [ ] 已验证 GNSS/视觉/LiDAR 单独失效时不会发布伪造状态或错误回退。
- [ ] 已完成 propeller-off 或系留测试，确认 ArduPilot EKF3 innovation、failsafe、
  外部导航源切换和 disarm 行为。
- [ ] 记录 EKF3 参数、ExternalNav topic、协方差、延迟补偿和 failsafe 事件。
- [ ] 只有在上述证据完整后，才允许扩大到真实飞行测试。

## 8. 结果与发布判定

- [ ] 所有“每次提交必做”项目通过。
- [ ] 所有受影响 ROS 2 包构建和测试通过，或已登记明确的环境阻塞原因。
- [ ] 仿真短路线通过，且没有重复 publisher、时间戳回退或进程泄漏。
- [ ] 视觉/HybridFusion 仅在功能启用时验证；未启用时确认默认路径不受影响。
- [ ] 真实 MID360 项目全部完成，或在发布说明中明确标记 `HARDWARE_DATA_REQUIRED`。
- [ ] 飞控 ExternalNav 项目全部完成后，才能把版本标记为可进行真实飞行验收。
- [ ] 将命令、commit、配置、原始日志、结果 JSON/CSV 和失败复现步骤保存到同一
  个验证目录，并在 PR 或 release note 中链接该目录。

