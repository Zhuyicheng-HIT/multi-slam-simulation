# Performance V2 稳定性与收敛

## 冻结决定

Performance V2 已冻结。早期 live-run 中 42.399 ms 与 54.375 ms 的差异来自 simulation host 调度，而不是可重复的 estimator 回归。冻结的 full-online input replay 在全部五次 V2 运行中均满足 `<= 45 ms` 门槛，run median 的中位数为 15.766 ms，运行间 CV 为 2.55%。没有修改 estimator threshold、sensor model、D_V/FRS 设置、0.065 s visual association tolerance、integrity check 或 rollback 行为。

完整 Gazebo wall-time 指标被标记为 `SIM_ENV_CONTENDED`。Performance profiling 仍需显式启用，CPU affinity 仅作诊断，不改变默认部署配置。

## live-run 差异的原因

精确的 transaction trace 将 estimator 工作与离开 CPU 的时间区分开。在相同 V2 实现和冻结输入下，solver P50 为 17.535 ms；在 Gazebo、RGB-D rendering、bridge、FAST-LIO、mapping 和 SITL 同时运行时为 33.081 ms。差异 15.546 ms，占 full-Gazebo P50 的 47.0%，大于之前观测到的 42/54 差异 11.976 ms。

full-Gazebo 中与 solver duration 相关性最强的是 voluntary context switch（`r=0.821`）和 involuntary context switch（`r=0.535`）。当 graph 仍为八个 state、通常 400 个 LiDAR correspondence 时，graph-linearization P50 从 replay 的 17.092 ms 增至 Gazebo 的 35.066 ms。同一数值工作量整体膨胀符合 preemption 特征，而非新增 factor 或 estimator 路径变化。Minor fault 仅为 `r=0.304`，major fault 为零。

被剖析的 Gazebo process group 使用整个 WSL CPU 容量的 6.906% 执行 SIM_ONLY 工作（Gazebo、simulation sensor bridge 和 SITL），命名的 REAL_TRANSFERABLE pipeline（backend、FAST-LIO、visual frontend、shared mapping）使用 5.919%。因此 SIM_ONLY 占这个不重叠命名 CPU 集合的 53.85%，并非全部 host CPU 工作的百分比。

Gazebo 无法打开 `/dev/dri/renderD128`，因为设备属于 `render` 组而 runtime 用户不在其中，EGL 因而使用 `kms_swrast`。虽然可见 `/dev/dxg` 与 NVIDIA 设备，已安装 OpenCV 未报告 CUDA 或 OpenCL 设备。未修改任何 group、driver 或 sudo 级系统设置。

## Transaction timing 分解

以下是五次通过运行中 2,874 个 V2 full-online replay transaction 的 P50/P95（单位 ms）。graph linearization 内部的嵌套计时存在重叠，不能简单相加。

| 阶段 | P50 | P95 |
|---|---:|---:|
| snapshot | 0.178 | 0.411 |
| state staging | 2.507 | 3.524 |
| IMU factor construction | 0.060 | 0.086 |
| NativeLidarFactor construction | 0.204 | 0.449 |
| GNSS factor construction | 0.045 | 0.062 |
| optical-flow factor construction | 0.007 | 0.566 |
| visual association | 0.034 | 0.056 |
| visual factor construction | 0.770 | 1.188 |
| graph assembly | 0.590 | 1.401 |
| graph linearization | 17.092 | 32.042 |
| LiDAR point-plane linearization subset | 5.991 | 10.807 |
| IMU preintegration linearization subset | 5.537 | 10.477 |
| marginal-prior linearization subset | 4.084 | 7.043 |
| visual reprojection linearization subset | 2.627 | 4.562 |
| linear solve | 0.657 | 1.403 |
| marginalization | 2.495 | 3.498 |
| integrity check | 0.145 | 0.256 |
| transaction commit | 0.807 | 2.422 |
| callback total | 22.450 | 39.876 |

solver 本身为 17.535/34.386 ms P50/P95。非 marginalizing startup cycle 为 11.520/21.721 ms（40 个 cycle）；marginalizing cycle 为 17.599/34.432 ms（2,834 个 cycle）。Marginalization 是正常且可见的开销，其稳定 cadence 无法解释 42/54 的运行差异。

## Factor 规模与运行时噪声

活动窗口中位数为 8 个 state、400 个 LiDAR correspondence 和 7 个 IMU factor。solver 与 state 数（`r=0.124`）、LiDAR correspondence（`r=-0.144`）、IMU factor（`r=0.090`）及 visual factor（`r=0.172`）的相关性都很小。Flow-factor 是否存在的相关性最大（`r=0.354`），但仍远小于 replay 中 voluntary context switch 的 `r=0.650`。

按规模归一化的 P50 为每个 LiDAR correspondence 0.0466 ms、每个 window state 2.202 ms。存在 active visual factor 的 transaction，其 solver P50 为 21.438 ms，无 visual factor 时为 17.295 ms；factor 是真实工作，但 cadence 低且固定，无法匹配全程方差。Trace 同时保存 active factor count 和每个 cycle 新增 factor count。

GC 未禁用。callback profiler 记录 generation collection、allocation count 和 duration。Replay GC duration 与 solver duration 的相关性仅 `r=0.058`，full Gazebo 为 `r=0.086`；每个 cycle 的 GC time 中位数和 P95 均为零，因此没有生产变更依据。WSL 中无法访问 CPU frequency 文件，按 unavailable 报告，不作推断。

## Full-online V1/V2 replay

冻结的 172.236 s 输入包含 83,125 条消息，覆盖 scan prediction/native-factor handshake、IMU、GNSS、flow、visual observation、D_V/reliability scheduling、pending association、八状态窗口、transactional integrity 和 marginalization。它有意排除 Gazebo、ArduPilot、camera rendering、map 与 ground truth，并保留在被忽略的 `logs/tmp` 树下。

| 指标 | 冻结 V1 | V2 |
|---|---:|---:|
| 每次运行 median (ms) | 19.546, 18.855, 18.844, 18.991, 18.821 | 15.766, 15.146, 15.465, 16.334, 15.899 |
| run median 的中位数 (ms) | 18.855 | 15.766 |
| run-median CV | 1.44% | 2.55% |
| pooled P50 (ms) | 19.667 | 16.916 |
| pooled P90 (ms) | 28.954 | 23.641 |
| pooled P95 (ms) | 31.303 | 25.969 |
| 每次 odometry 消息数 | 574--576 | 574--576 |

V2 使匹配的 run median 提升 16.38%，pooled P95 提升 17.04%。Transaction trace 中 optimization error、integrity reject 和 rollback 均为零。每次运行接受 11 个 visual factor。少量重复 native worker input 由现有 bounded queue 合并；这不是 integrity failure，也不改变最终 estimator state。

## CPU affinity 诊断

在同一 replay 上，正常调度的 solver P50/P95 为 17.839/35.528 ms。仅将 backend 固定到 CPU 30 后为 13.711/29.795 ms，分别改善 23.1% 和 16.1%。每 cycle 的 voluntary context switch 中位数从 171.5 降至 11.0；involuntary switch 从 0 增至 8，因为单个隔离逻辑 CPU 仍可能被抢占。正确性保持 zero-error、zero-integrity-reject、zero-rollback。

Full-stack affinity probe 将 Gazebo solver P50 从 33.081 降至 31.477 ms，P95 从 73.364 降至 54.679 ms。该 probe 不是 correctness evidence：其 flight 遇到 8 次早期 bias-integrity rollback，flight 后等待 vision topic 直到 bounded cleanup 才结束。Affinity 脚本现会持续分配晚启动的 descendant，而不是只做一次早期 process snapshot。Affinity 仅保留作可复现诊断，不是部署要求。

## 保留的正确性门槛

最终接受的三个 rectangle 和三个 S-curve 仍是 correctness set：candidate-to-solver acceptance 为 71.81%，time rejection 为 10.28%，optimization error、integrity reject 和 rollback 均为零。之前匹配的五次 V1/V2 rectangle 对比未发现系统性精度回归。已接受的 joint-map run 包含 108,191 个 voxel、10.41% occupied-volume growth、12.50% color coverage、零 geometry conflict 和零 eviction；LiDAR 仍是 geometry authority。

新的 performance-only instrumentation 默认禁用。仅在明确请求时写入有界 JSONL trace，记录 process fault、context switch、CPU/RSS、GC 和 process-group load，不改变 factor selection 或 solver math。

最终验证：

- 15 个 package 以 `RelWithDebInfo` 构建；
- 57 个 colcon result file，error、failure、skip 均为零；
- backend 160/160 与 visual frontend 4/4 direct test 通过；
- 最终 trace-schema smoke replay：576 个 cycle、576 条 odometry 消息、11/11 个 visual factor，solver P50/P95 为 17.108/34.370 ms，correctness failure 为零；
- D435i active-run lifecycle 短测试通过；
- Python 198、YAML 29、XML 15、shell 53 项检查通过；
- `git diff --check` 通过；
- 所有自有 runtime process 均清理；
- 冻结 V1 仍为 `d76543e9c8f80dcaecbcbe4d898811a420978094`，源代码无修改。
