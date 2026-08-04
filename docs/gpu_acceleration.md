# GPU acceleration boundary

This project uses the GPU where Gazebo and the installed libraries provide a
maintained backend. It does not label CPU code as GPU-accelerated merely because
an NVIDIA device is visible in WSL.

## Active GPU path

On WSLg hosts with an NVIDIA GPU, `scripts/env.sh` selects the NVIDIA D3D12
adapter with `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`. Gazebo continues to use
OGRE2. The following simulated sensors therefore render on the selected GPU:

- optical-flow camera;
- optical-flow `gpu_lidar` range sensor;
- D435-style RGB-D camera;
- MID360 `gpu_lidar`.

The MID360 sensor remains enabled, but its Gazebo GUI visualization is disabled.
This removes a display-only copy without changing the scan, noise, range, rate,
or ROS topics.

Run the capability probe directly:

```bash
cd "$HOME/multi-slam-github-staging"
bash tools/check_gpu_acceleration.sh
```

For evaluation runs, reject silent software or wrong-adapter fallback:

```bash
bash tools/run_sim_gpu_headless.sh
```

`HEADLESS=1` uses Gazebo's OGRE2 EGL path. It removes the GUI process but does
not disable camera, depth, or GPU LiDAR sensors. The GPU evaluation wrapper also
disables the D435 ROS conversion bridge by default because the current LIO/flow
stage does not consume those images. The simulated D435 sensor remains present;
set `ENABLE_D435_BRIDGE=1` when RGB-D topics are required.

## Local controlled check

The following short runs used the same world, headless OGRE2, optical flow,
MID360 bridge, D435 point cloud disabled, and no FAST-LIO process:

| Adapter / bridge profile | MID360 ROS rate | Main observed CPU loads |
| --- | ---: | --- |
| AMD Radeon 610M, D435 bridge on | about 4.57 Hz | Gazebo 164%, D435 bridge 107% |
| RTX 5060, D435 bridge on | about 5.1 Hz | Gazebo 188%, D435 bridge 105% |
| RTX 5060, D435 bridge off | about 7.52 Hz | Gazebo 183%, MID360 bridge 63% |

Linux `%CPU` is per logical core, so 183% means about 1.83 fully occupied CPU
cores. This result confirms both that RTX selection helps and that the remaining
rate limit is CPU physics plus serialized Python bridges, not GPU saturation.
The NVIDIA snapshot during the RTX runs was roughly 5-9% utilization and about
1.9 GiB VRAM. These are startup/stationary checks, not a flight-quality baseline.

## Full rectangle validation (2026-07-26)

The RTX headless profile also completed the repository's default 2.0 x 1.2 m
rectangle with image-based optical flow (`FLOW_USE_PHYSICS=false`), FAST-LIO,
ArduPilot, MAVROS, and the drift analyzer running together for 125 seconds.

- Analyzer result: `passed=true`, 958 matched odometry/cloud samples.
- Position RMSE / max / final: 0.0616 / 0.0974 / 0.0369 m.
- Yaw RMSE / max / final: 0.102 / 0.477 / 0.0856 degrees.
- FAST-LIO yaw-rate vs FCU gyro correlation: 0.878.
- Estimated FCU IMU lag: 20 ms.
- Raw, registered-cloud, and FCU-IMU timestamp regressions: zero.
- Cloud stamp period median / p95: 127.5 / 146.8 ms (about 7.8 Hz median).
- Voxel overlap p05 / median: 0.387 / 0.514.
- Cloud centroid jump p95 / max: 2.98 / 6.55 m.
- During the full run, RTX utilization averaged 8.3% (maximum 9%), VRAM averaged
  1.98 GiB, Gazebo averaged 198% CPU, both point-cloud Python bridges together
  averaged about 96% CPU, and FAST-LIO averaged 20% CPU.

This route is smaller than the older 6 x 4 m reference route, so its trajectory
errors must not be presented as a direct improvement over that baseline.

The same run did not pass the optical-flow accuracy gate: correlation was 0.855,
but estimated scale was 0.630 and normalized RMSE was 0.501. GPU selection is
validated; image-flow scale/calibration remains a separate open issue and blocks
publishing this state as an optical-flow milestone.

## CPU-only boundary

The following code remains on the CPU in the current dependency set:

- ArduPilot SITL and Gazebo physics;
- FAST-LIO IKFoM, PCL filters, incremental KD-tree, and native residual export;
- ROS 2 / Gazebo message serialization and the Python MID360 bridges;
- GTSAM/Ceres backend code unless a separately validated CUDA solver is added;
- LK optical flow.

The installed Python OpenCV 4.10 build reports zero CUDA devices through OpenCV,
has no CUDA SparsePyrLK binding, and has no usable OpenCL runtime. The RTX is
visible to WSL, but this OpenCV binary was not built with CUDA. A CUDA optical
flow migration therefore requires a pinned custom OpenCV + opencv_contrib build,
an isolated environment, and numerical/performance regression tests. For the
current 100 x 100, 30 Hz flow stream, this is deferred because transfer and
launch overhead may exceed the LK compute saved.

## Official references

- Gazebo Sim headless rendering: https://gazebosim.org/api/sim/8/headless_rendering.html
- Gazebo Rendering engines: https://gazebosim.org/api/rendering/8/installation.html
- Microsoft WSL multi-GPU adapter selection: https://learn.microsoft.com/en-us/windows/wsl/tutorials/gpu-compute
- OpenCV CUDA runtime introduction: https://docs.opencv.org/4.x/d2/dbc/cuda_intro.html
- OpenCV CUDA optical flow: https://docs.opencv.org/4.x/d7/d3f/group__cudaoptflow.html
