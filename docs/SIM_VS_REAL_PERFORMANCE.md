# 仿真与真实硬件性能

## 可迁移的计算开销

边缘先验 block transform、transaction snapshot reduction、batched analytic
visual factor、batched voxel indexing、结构化 PointCloud2 packing、有界诊断
以及 `RelWithDebInfo` build 在伴随计算机上保持不变。因此，实测 estimator、
visual frontend、reliability、mapping、ROS transport 和 memory/copy 开销可用于硬件选型。

## 仅仿真开销

Gazebo rendering/physics、gz-to-ROS bridges、ArduPilot SITL 和 WSL scheduling
不会迁移到飞行器。在代表性配置中，Gazebo 单独约占 WSL 总容量的 5.2%，RSS
约 0.54 GiB；bridges 约占 1.2%，SITL 约占 0.18%。

该机器暴露了 `/dev/dxg` 和 RTX 4070，但 OpenCV 报告 CUDA 设备数为零且没有
OpenCL。EGL 无法打开 `/dev/dri/renderD128`（permission denied），并回退到
`kms_swrast`；`glxinfo` 不可用。无头渲染已经启用。修复设备权限或 WSL 图形
栈需要主机/系统更改，因此有意未使用 `sudo` 尝试。

因此，solver 改进是有效的算法结果，而小幅 live RTF 变化必须标注为 SIM_ONLY，
不得推断到飞行器。一次短时确定性的 backend replay 可以进一步隔离 estimator，
但仓库未提交新的 rosbag，也未报告虚构的 replay 结果。
