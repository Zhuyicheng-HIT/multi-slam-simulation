# 稳定 MicoLink 基线

本基线基于 `5b3e36a`（`feat: support multiple replay datasets`）。保留已验证的
MicoLink optical-flow protocol 和 replay viewer，同时排除后续大型场景重定位及
纹理隧道实验。

## 包含内容

- MicoLink COM17 optical-flow framing、parser、unit conversion 与 tests；
- 冻结的 five-source backend 及其低空 ExternalNav contract；
- 基础提交中已有的 dataset replay 和可 seek visualization tools；
- 着陆时 observer shutdown 与 ROS clock-regression 处理；
- MAVROS validation plugin 列表，默认不包含 `param` plugin；
- 使用一个 FCU IMU 作为 simulation IMU source；optical-flow camera 与 range
  sensor 保持启用，同时移除 optical-flow 和 D435i IMU；
- MID360 仿真降为水平 360、垂直 16 个采样点。

## 排除内容

- 大型隧道纹理和直线路线实验；
- 被动/主动重定位 campaign 的改动；
- RGB-D hard-depth admission 与 photometric downweighting 实验；
- directional handoff、dynamic-scene 及其他未经验证的基线后算法实验。

## 验证

- `colcon build --symlink-install --packages-select multi_slam_uav_sim`；
- `colcon test --packages-select multi_slam_uav_sim`：129 tests passed；
- `colcon test-result --verbose`：76 tests，0 errors，0 failures，0 skipped；
- Shell syntax、XML parsing、sensor contract 和 `git diff --check` 均通过。

与早期 checkpoint 关联的历史 frozen-server reference 在记录的 validation log
中约为 3.14 cm causal 3D RMSE。本文件仅记录来源，不声称复现新的飞行结果。
