# PR21 Runtime Graph 合并审计

基线：PR20 `5f588dcbfb8d1f61e3335bd47d6c74457421666f`

## 进程边界

Optimizer、FAST-LIO、relocalization compute node 和 ExternalNav safety gate 仍是独立进程，拥有各自的故障处理；factor/topic 语义不变。Reliability monitor 与 scheduler 也保持分离，scheduler 消费 monitor 分数，避免一个诊断组件故障拖垮另一个组件。

## 生产合并

`sensor_relay_manager` 是一个多线程 rclpy 进程，只为启用的 modality 执行现有 relay/unit-normalization。它保留 sensor-data QoS 和所有公共 topic。旧的 `fault_injector_*` 仅用于测试，只有 `enable_fault_injection:=true` 或显式 legacy fault 环境存在时才启动；生产模式不再启动 6 个空闲 Python injector。

Body point filter、GNSS metadata association、reliability nodes、LIO adapter、backend、relocalization 与 ExternalNav gate 继续拥有各自 topic，因为它们承担不同工作或属于安全边界。

## 配置 profile

| Profile | Sensor relay modality | Vision | Fault injector |
|---|---|---|---|
| `minimal_lidar_imu` | LiDAR、IMU | off | off |
| `four_source` | LiDAR、IMU、GNSS、Flow | off | off |
| `five_source` | LiDAR、IMU、GNSS、Flow、RGB-D | on | off |
| `robustness` / `test` | LiDAR、IMU、GNSS、Flow、RGB-D | on | on |

Backend scheduler 继续使用既有 modality 名称（RGB-D 使用 `vision`），relay manager 仅在传输层使用 `depth` 和 `color`。调用方可继续使用原 topic 与 launch 文件。

## 资源与端点预期

生产 four-source profile 用一个 relay 进程替代 6 个 modality relay（减少 5 个 Python 进程），DDS graph 删除 5 组重复订阅/发布。高频 topic 在 manager 中每个启用 modality 只复制一次；minimal 模式还移除 GNSS association 和 Flow relay；robustness 模式有意保留隔离 injector 以便故障归因。

没有新增高频 timer 或 estimator callback。manager 只使用 message callback，两个 executor thread 服务独立传感器回调。CPU/RAM 与启动时间必须在目标 Intel 计算机上测量；本改动只保证 graph 精简，不宣称开发机硬件指标。

## 兼容性与正确性

原始输入 topic 不变，公共 normalized topic 与 QoS 不变，IMU acceleration normalization 继续使用原 SI conversion helper；没有修改 fusion factor、weight、HXY 或 one-observation-one-factor 逻辑。Fault-injection launch 仍可作为显式 test profile 使用。
