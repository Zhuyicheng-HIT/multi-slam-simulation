# Main 验证进度

本文档记录已合并 `main` 分支上的验证结果，并区分软件证据、仿真验收和硬件验收。本文档是进度记录，不代表发布或真实飞行验收声明。

## 基线信息

- 日期：`2026-09-14`
- Commit：`3f3974c`（`test: rebuild live propagation measurement after anchor retry`）
- Branch：`main`
- ROS：WSL 中的 ROS 2 Humble，Ubuntu 22.04.5
- Gazebo：`8.15.0`
- CPU/RAM：AMD EPYC 9354，64 个逻辑 CPU，62 GiB RAM
- GPU：NVIDIA RTX PRO 6000，WSLg direct rendering 已启用
- External versions：
  - ArduPilot：`f9d619e26002d6aaa41643ee99c0ae0ee01e2247`
  - ArduPilot Gazebo：`082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
  - Livox ROS Driver 2：`13eb05e4e6dd7a765b934d0c5fd6236676a57b49`
  - FAST-LIO：`a4743b095409588842a5b30ddfa27e29d2f99164`

## 已通过

### 静态检查与 CI

- GitHub Actions CI 已通过，验证的是前一个代码 commit `873e3ea`。
- `tools/ci_smoke_test.py`：`33 passed`。
- 全部已跟踪 Python 文件通过 `py_compile`。
- 全部已跟踪 Shell 文件通过 `bash -n`。
- 44 个已跟踪 YAML/YML 文件解析成功。
- 41 个已跟踪 XML/SDF 文件解析成功。
- `git diff --check` 通过。
- 已跟踪源码中没有未解决的 merge conflict 标记。

### Build 与 unit test

- 完整 `colcon build --symlink-install`：20 个 package 全部完成。
- `uf_map_maintenance` 和 `uf_global_pose_graph` 的 build 与 test 通过。
- `mid360_sim_bridge_cpp`：13 项 C++ test 通过。
- `uf_reliability`：单独运行时 81 项 test 通过。
- 修复 anchor-retry test 后，`uf_backend_fusion`：301 项 test 通过。
- 受影响 package 的 `colcon test-result`：148 tests，0 errors，0 failures，0 skipped。

### HybridFusion deterministic benchmark

三轮 benchmark 已完成，结果目录：

`logs/hybridfusion/benchmark_validation_20260914/`

- Initial：3/3 converged。
- GICP：3/3 converged。
- Hybrid：3/3 converged。
- 9 次 method run 全部以 exit code 0 结束。
- 汇总文件：`logs/hybridfusion/benchmark_validation_20260914/summary.md`。

这是基于 deterministic generated data 的 benchmark，不能作为真实传感器精度证据。

## 部分验证

- 第一次完整 `colcon test` 发现一个确定性 `live_propagation` anchor-retry test 失败。原因是 test double 在 anchor commit 后复用了旧 IMU measurement。
- 已将 test double 改为基于新 anchor 重新生成匹配的 measurement，随后受影响的 `uf_backend_fusion` package test 通过。
- 测试修复后没有重新执行完整工作区 `colcon test`；目前记录的是受影响 package 的复测结果，建议后续进行一次干净的 full-suite rerun。
- Livox、FAST-LIO、ArduPilot SITL 和 ArduPilot Gazebo 依赖均已存在，并可在 source 对应 overlay 后被发现；本轮没有使用真实传感器数据。

## 尚未验证

### Gazebo 与 ArduPilot runtime

- 当前 commit 尚未完成一次干净的 short rectangle/flight route。
- 尚未从干净运行中记录统一 sensor topic、`/clock` single ownership、frame ID、frequency、timestamp monotonicity 和 packet loss。
- 尚无 runtime log 证明 FAST-LIO 正在消费并连接到 unified backend。
- Unified odometry writer ownership 和 5–10 分钟 static replay 尚未完成。

第一次启动尝试直接调用 `src/multi_slam_uav_sim/scripts/run_apm_sensor_stack.sh` 时暴露了 script invocation/path 问题：脚本将 workspace install directory 解析为 `/home/ld666`，并尝试 source 不存在的 `/home/ld666/setup.bash`。在 runtime acceptance 前，需要使用正确的 installed-workspace entrypoint 或修正 path resolution。

### Visual 与 live HybridFusion

- Deterministic offline HybridFusion 已通过。
- Live RGB-D/CameraInfo/TF pairing、live exporter 行为，以及对 FAST-LIO/RTAB-Map 的 non-interference 尚未测试。

### Hardware 与 flight

- 真实 MID360 静止和运动测试：`HARDWARE_DATA_REQUIRED`。
- IMU unit scale、device/ROS time alignment、network loss、driver reconnect、external-navigation EKF3 和 failsafe 行为：`HARDWARE_DATA_REQUIRED`。
- 不作真实飞行验收声明。

## 下一步

1. 修复或统一 `run_apm_sensor_stack.sh` 的 installed-workspace entrypoint。
2. 在只有一个 sensor bridge 的干净环境中运行 headless Gazebo/ArduPilot short rectangle，并保存 raw log 与 topic audit。
3. 干净地运行 full `colcon test`，并保存 `colcon test-result --verbose`。
4. 运行 unified-odometry static replay，检查 anchor、timestamp、queue discard 和 native worker diagnostics。
5. 如果启用 visual profile，执行 live HybridFusion validation。
6. 只有在真实硬件可用时，才执行 MID360 和 ExternalNav validation，并保存 sensor、timing、covariance 和 failsafe 证据。

## 相关方案

详细检查清单见 [`VALIDATION_PLAN.md`](VALIDATION_PLAN.md)。
