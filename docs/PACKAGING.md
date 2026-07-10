# GitHub 打包原则

本仓库以源码为核心，并保证可从公开上游依赖复现。原则是：项目自己维护、体积合理且复现必需的内容进入仓库；可公开下载的大型软件和本机生成内容不进入仓库。

## 1. 纳入仓库

- 项目自研 ROS 2 包与节点；
- SDF 场景和项目专用的小型传感器模型；
- 飞控参数、RViz 配置、launch 文件和启动脚本；
- 项目自研的 MID360 可靠建图节点；
- 外部依赖地址、固定提交号、安装和运行说明；
- 用于检查路径、大文件和生成目录的发布工具。

## 2. 不纳入仓库

- `build/`、`install/`、`log/`、运行日志和缓存；
- ArduPilot 源码、子模块和 SITL 二进制；
- Gazebo 安装包、用户缓存和已编译插件；
- 完整的 `ardupilot_gazebo`、FAST-LIO、Livox 驱动与 Livox-SDK2 仓库；
- 可直接下载的 Clearpath 与地形生成器仓库；
- rosbag、PCD、地图、视频、截图等运行生成文件；
- 当前场景未使用的上游 Iris 网格副本。

## 3. 为什么这样打包

- 避免把数 GB 的安装目录和编译产物上传到 GitHub；
- 避免不同 Ubuntu、ROS 2 或 CPU 架构之间复用错误二进制；
- 通过固定提交号保证外部依赖可追溯；
- 保持上游项目许可证和项目自研代码边界清晰；
- 减少个人路径、缓存与运行数据泄漏。

## 4. 路径规则

项目文件使用 ROS 2 package share、脚本相对路径或 `<工作空间>` 占位符。外部目录只通过以下环境变量配置：

```text
ARDUPILOT_DIR
ARDUPILOT_GAZEBO_DIR
LIDAR_WS
MULTI_SLAM_EXTERNAL_DIR
```

不得提交 `/home/某个用户名/...` 等个人绝对路径，也不得提交指向本机目录的绝对符号链接。

## 5. 发布检查

每次提交或发布前执行：

```bash
python3 tools/verify_repository.py
git status --short
git ls-files
```

检查 Git 实际跟踪内容，而不只是查看工作目录。确认没有大型二进制、编译目录、日志、个人绝对路径或失效符号链接。
