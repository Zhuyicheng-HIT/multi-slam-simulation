# D435i RGB-D 视觉 SLAM 状态

## 当前状态

本分支是 WIP/Draft 基线，目标是先提供可审查、可复现的纯视觉链路。当前已
完成：

- 高性能 C++ RGB-D bridge：成对发布、共享时间戳、CameraInfo、optical
  frame、`16UC1`、可配置 QoS、按需 PointCloud2；
- Python bridge 兼容降级；
- RTAB-Map `feature_aligned` profile、exact sync、headless launch、唯一
  `/clock` 检查；
- D435i-only 模型、基础/纹理世界、精确 PID 清单和停止脚本；
- bridge/RTAB 性能与延迟、ATE/RPE、只读数据库、A-G 航线、鲁棒性和速度
  包线工具。

## 已验证行为

- 640×480 RGB-D 在 D435i-only 基线中约 28–29 Hz；
- RTAB-Map 在正式运动样本中约 16 Hz；
- `Vis/FeatureType=6` 与 `Kp/DetectorStrategy=6` 对齐后，视觉词袋和
  GlobalClosure 链路已验证；
- 推荐后续测试水平速度为 0.35 m/s；
- 0.75 m/s 直线独立有效样本 3/3 PASS；
- 已记录的有效样本没有 lost、reset、TF backward 或错误闭环；
- 原完整 GUI 仿真的 MID360、光流、MAVROS 和 D435i 默认入口仍可启动。

历史性能数值来自稳定提交 `727d6e0` 的已完成实验记录。移植到最新
`main` 后重新执行了语法、构建和静态检查；本 Draft 不把历史原始日志或
数据库带入仓库。详细口径见 benchmark 文档。

## 兼容性边界

- 原 `run_apm_sensor_stack.sh` 默认仍启动原完整传感器栈；
- 上游最新 main 的 MAVROS IMU 请求速率 100 Hz 保留；
- D435i-only profile 的开关不会改变 FAST-LIO、MID360 mapper 或
  Ultra-Fusion 代码；
- RTAB-Map 位姿只用于评估，未接入飞控闭环或多源融合后端。

## 已知限制

- 当前 Gazebo 渲染仍使用 `kms_swrast`；GPU render node 权限问题不在本
  PR 范围内；
- 尚未在真实 D435i、真实 USB 链路和真实飞行器上验证；
- 仿真不完整建模曝光、rolling shutter、运动模糊、振动和真实传输抖动；
- 0.75 m/s 是当前场景的已验证压力档，不代表实机安全速度；
- 短矩形或短回环路线可能无法持续达到高指令速度，必须标为
  `NOT_EXERCISED`，不能冒充 PASS 或 FAIL。

## 后续计划

后续稳定成果继续追加到同一分支和同一 Draft PR：

1. 只从正式提交读取已完成、已验证的阶段成果；
2. 重新基于最新 `origin/main` 检查冲突；
3. 重跑构建、测试、D435i-only smoke 和完整仿真回归；
4. 追加独立 commit，并同步更新 PR 验证表。

本 Draft 不包含实机验证、多源融合或未完成的阶段 A/B/C 工作树内容。
