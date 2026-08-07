# Gazebo Sim 场景说明

`multi_slam_worlds` 收集适用于 Gazebo Sim Harmonic 的多源 SLAM 与无人机导航测试场景。

主 APM 多传感器仿真使用
`multi_slam_uav_sim/worlds/simple_apm_rgbd_mid360.sdf`。该世界除纹理地面外，还加载
`s_curve_urban_structures`：静态拱门、短隧道、分段高墙走廊和不对称建筑立面。
这些结构采用简单 box collision/visual，优先保证 LiDAR 几何可观测性和实时因子，
不引入大型网格资源。

## 1. 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select multi_slam_worlds
```

## 2. 加载环境

```bash
source install/setup.bash
source install/multi_slam_worlds/share/multi_slam_worlds/scripts/env.sh
```

## 3. 场景启动命令

简单无人机测试地图，包含纹理地面、墙体与基础障碍物：

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh simple_test
```

激光雷达隧道退化场景：

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh tunnel
```

适合后续 APM SITL 联调的 ArduPilot 仓库：

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh ardupilot_warehouse
```

Clearpath 室内仓库障碍物场景：

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh clearpath_warehouse
```

Clearpath 办公室走廊与重定位场景：

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh office
```

Clearpath 施工环境密集障碍场景：

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh construction
```

Gazebo Terrain Generator 的 Apple Park 城市级地形：

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh city_applepark
```

Gazebo Terrain Generator 的 Joshimath 城镇地形：

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh city_joshimath
```

## 4. 外部场景依赖

Clearpath 与地形生成器相关场景需要额外下载对应上游仓库，默认简单场景、隧道和 ArduPilot 仓库不需要这些大型依赖。下载位置与环境变量见 [安装说明](INSTALL.md)。

## 5. 验证说明

精选场景已在 Gazebo Sim 8.13.0 中使用以下无界面命令进行冒烟测试：

```bash
gz sim -s -r --headless-rendering \
  "$HOME/projects/multi-slam-simulation/src/multi_slam_worlds/worlds/simple_uav_test.sdf"
```

仓库只保留本项目选定的场景入口；可公开下载的大型场景仓库和生成地形不重复打包。

## 6. 主场景固定航线边界

`tools/run_s_curve_state_machine.sh` 使用三维长 S 航线穿越主场景，默认速度
`0.35 m/s`，每约 `2 m` 停留并等待融合定位收敛，飞行高度在统一后端起飞原点的
`-1.0..+1.0 m` 范围内完成两次升降。航线从中心沿曲线进入端点，完成往返后再沿
曲线回到中心；禁止恢复旧的直线进场/返场，因为它会穿过短隧道侧墙。

控制真值边界如下：

- 航线目标、位置误差、航点到达和任务完成只使用 `/fusion/unified/odom`；
- `/mavros/local_position/pose` 仅作为 APM local setpoint 的坐标表达适配器；
- Gazebo ground truth 只用于飞行后的 ATE/RPE 和碰撞评估；
- 统一后端缺失、过期或 frame 不是 `camera_init -> body` 时，任务在解锁前失败，
  不自动切换为 FCU/Gazebo 导航。

静态碰撞审计覆盖标定八字、曲线进场、全部 S 往返和曲线返场。审计图及 JSON：

```bash
cd "$HOME/projects/multi-slam-simulation"
python3 tools/plot_s_curve_world_audit.py \
  --output docs/assets/s_curve_world_audit.png \
  --json docs/assets/s_curve_world_audit.json
```
