# GPU 加速边界

本项目仅在 Gazebo 和已安装库提供受维护后端时使用 GPU。即使 WSL 能看到
NVIDIA 设备，也不会仅凭此将 CPU 代码标记为 GPU 加速。

## 当前 GPU 路径

在带 NVIDIA GPU 的 WSLg 主机上，`scripts/env.sh` 通过
`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 选择 NVIDIA D3D12 适配器。Gazebo
继续使用 OGRE2。因此，下列仿真传感器会在选定 GPU 上渲染：

- 光流相机；
- 光流 `gpu_lidar` 距离传感器；
- D435 风格 RGB-D 相机；
- MID360 `gpu_lidar`。

MID360 传感器仍保持启用，但关闭 Gazebo GUI 中的可视化。这会移除仅用于
显示的复制操作，不改变扫描、噪声、量程、频率或 ROS topic。

直接运行能力探测：

```bash
cd "$HOME/multi-slam-github-staging"
bash tools/check_gpu_acceleration.sh
```

评估运行时拒绝静默的软件渲染或错误适配器回退：

```bash
bash tools/run_sim_gpu_headless.sh
```

`HEADLESS=1` 使用 Gazebo 的 OGRE2 EGL 路径。它会移除 GUI 进程，但不会
禁用相机、深度或 GPU LiDAR 传感器。GPU 评估封装脚本默认也会关闭 D435
ROS 转换桥，因为当前 LIO/光流阶段不消费这些图像。仿真 D435 传感器仍然
存在；需要 RGB-D topic 时设置 `ENABLE_D435_BRIDGE=1`。

## 本地受控检查

以下短时运行使用相同场景、无头 OGRE2、光流、MID360 bridge、禁用 D435
点云，且不启动 FAST-LIO 进程：

| 适配器 / bridge 配置 | MID360 ROS 频率 | 主要观测 CPU 负载 |
| --- | ---: | --- |
| AMD Radeon 610M，开启 D435 bridge | 约 4.57 Hz | Gazebo 164%，D435 bridge 107% |
| RTX 5060，开启 D435 bridge | 约 5.1 Hz | Gazebo 188%，D435 bridge 105% |
| RTX 5060，关闭 D435 bridge | 约 7.52 Hz | Gazebo 183%，MID360 bridge 63% |

Linux `%CPU` 按逻辑核心计数，因此 183% 约等于 1.83 个满载 CPU 核心。
结果表明 RTX 选择确实有帮助，剩余的频率限制来自 CPU physics 和串行的
Python bridge，而不是 GPU 饱和。RTX 运行期间 NVIDIA 监控约为 5–9% 利用率、
约 1.9 GiB VRAM。这些是启动/静止检查，不是满足飞行质量的基线。

## 完整矩形验证（2026-07-26）

RTX 无头配置还完成了仓库默认的 2.0 x 1.2 m 矩形轨迹，同时运行基于图像
的光流（`FLOW_USE_PHYSICS=false`）、FAST-LIO、ArduPilot、MAVROS 和漂移
分析器，持续 125 秒。

- 分析器结果：`passed=true`，匹配 958 个里程计/点云样本；
- 位置 RMSE / 最大值 / 最终值：0.0616 / 0.0974 / 0.0369 m；
- 偏航 RMSE / 最大值 / 最终值：0.102 / 0.477 / 0.0856 度；
- FAST-LIO 偏航速率与 FCU gyro 的相关系数：0.878；
- 估计 FCU IMU 延迟：20 ms；
- 原始点云、注册点云和 FCU-IMU 时间戳回归：均为零；
- 点云时间间隔中位数 / p95：127.5 / 146.8 ms（中位频率约 7.8 Hz）；
- 体素重叠 p05 / 中位数：0.387 / 0.514；
- 点云质心跳变 p95 / 最大值：2.98 / 6.55 m；
- 完整运行期间，RTX 平均利用率 8.3%（最大 9%），VRAM 平均 1.98 GiB，
  Gazebo 平均 CPU 负载 198%，两个点云 Python bridge 合计约 96%，FAST-LIO
  约 20%。

此路线小于旧的 6 x 4 m 参考路线，因此不能把其轨迹误差直接表述为相对
该基线的改进。

同一次运行未通过光流精度门限：相关系数为 0.855，但估计尺度为 0.630、
归一化 RMSE 为 0.501。GPU 选择已得到验证；图像光流的尺度/标定仍是独立
的开放问题，因此不能将当前状态作为光流里程碑发布。

## 仅 CPU 的边界

在当前依赖集合中，下列代码仍运行在 CPU：

- ArduPilot SITL 与 Gazebo physics；
- FAST-LIO IKFoM、PCL filters、增量 KD-tree 和原生 residual export；
- ROS 2 / Gazebo 消息序列化与 Python MID360 bridges；
- GTSAM/Ceres backend，除非另行加入并验证 CUDA solver；
- LK optical flow。

已安装的 Python OpenCV 4.10 构建通过 OpenCV 报告 CUDA 设备数为零，没有
CUDA SparsePyrLK binding，也没有可用的 OpenCL runtime。虽然 WSL 能看到
RTX，但该 OpenCV binary 未启用 CUDA。因此迁移到 CUDA optical flow 需要
固定版本的自定义 OpenCV + opencv_contrib 构建、隔离环境以及数值/性能回归
测试。对于当前 100 x 100、30 Hz 的光流流，传输与 kernel 启动开销可能超过
节省的 LK 计算量，故暂缓迁移。

## 官方参考

- Gazebo Sim 无头渲染：https://gazebosim.org/api/sim/8/headless_rendering.html
- Gazebo Rendering engines：https://gazebosim.org/api/rendering/8/installation.html
- Microsoft WSL 多 GPU 适配器选择：https://learn.microsoft.com/en-us/windows/wsl/tutorials/gpu-compute
- OpenCV CUDA runtime 简介：https://docs.opencv.org/4.x/d2/dbc/cuda_intro.html
- OpenCV CUDA optical flow：https://docs.opencv.org/4.x/d7/d3f/group__cudaoptflow.html
