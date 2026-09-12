# 稳定室内基线 2026-08-24

本基线基于 `checkpoint/stable-microlink-20260822`，仅包含启动修复，未改变
fusion algorithm 或 sensor weighting 配置。

## 验证

使用普通室内场景验证：

```bash
LOG_DIR="$PWD/logs/stable_baseline_indoor_aug22_rerun_20260824" \
VALIDATION_WORLD_PATH="$PWD/src/multi_slam_uav_sim/worlds/low_indoor_apm_rgbd_mid360.sdf" \
VALIDATION_WORLD_NAME=low_indoor_apm_rgbd_mid360 \
VALIDATION_GAZEBO_WORLD_NAME=low_indoor_apm_rgbd_mid360 \
VALIDATION_ROUTE=rectangle \
VALIDATION_ROUTE_FEEDBACK_SOURCE=fcu_local \
VALIDATION_LOCALIZATION_SAFETY_ENABLED=true \
VALIDATION_ENABLE_EXTERNALNAV_EKF3=0 \
VALIDATION_ENABLE_VISION=1 \
VALIDATION_RECORD_REPLAY_BAG=false \
VALIDATION_RECORD_RAW_LIDAR=false \
VALIDATION_REQUIRE_FASTLIO_DRIFT=false \
VALIDATION_STOP_OBSERVERS_ON_LANDING=true \
VALIDATION_MINIMUM_SIM_DURATION=0 \
RECTANGLE_LENGTH_X=2 RECTANGLE_LENGTH_Y=1.2 \
RECTANGLE_SPEED=0.2 RECTANGLE_HOLD_TIME=2 \
bash tools/run_unified_rectangle_validation.sh
```

飞行器完成起飞、矩形四条边、着陆和 disarm，严格验证结果为 `passed=true`。

## 精度

指标采用冻结的初始对齐并具有因果性：

| 指标 | 结果 |
| --- | ---: |
| 3D RMSE | 0.0245 m |
| 3D P95 | 0.0346 m |
| 3D 最大值 | 0.0523 m |
| XY RMSE | 0.0147 m |
| Z RMSE | 0.0196 m |
| 终点误差 | 0.0211 m |

本次运行使用 unified five-source backend，并关闭 ExternalNav FCU consumption，
用于 estimator-only 评估。LiDAR、GNSS、optical flow、RGB-D 和 IMU factor 路径
均启用；native factor queue 丢失和 optimization rollback 计数均为零。

## 启动修复

- 在等待 LiDAR bridge 前等待 MAVROS IMU 与 simulated barometer，避免 bridge 初始化死锁；
- 加载 `mavros_apm_rgbd.yaml`，使 MAVROS 连接 SITL TCP endpoint；
- 使用依赖工作空间的 `local_setup.bash`，保留 ArduPilot Gazebo system plugin 路径。

修改后的构建与测试验证：

```text
colcon build --symlink-install: passed
colcon test-result --verbose: 76 tests, 0 errors, 0 failures
```
