# Gazebo Sim 场景说明

`multi_slam_worlds` 收集适用于 Gazebo Sim Harmonic 的多源 SLAM 与无人机导航测试场景。

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
gz sim -s -r --headless-rendering <场景文件>
```

仓库只保留本项目选定的场景入口；可公开下载的大型场景仓库和生成地形不重复打包。
