# Runtime Lightweight Audit

Date: 2026-07-20

## Storage audit

| Path | Size | Decision |
|---|---:|---|
| `src/` | 1.3 MB | Keep; source code is not the storage problem. |
| `build/` | 36.5 MB | Keep; symlink installs and incremental builds depend on it. |
| `install/` | 0.8 MB | Keep; required at runtime. |
| `log/` | 31.7 MB | Regenerable colcon output; safe cleanup candidate. |
| `logs/` | 3007.3 MB | Keep milestone data; apply selective retention only. |

The largest artifact is
`logs/uf_stage2_20260713_102507/bag/sensors/sensors_0.db3` at 2883.8 MB.
It is a 224 second, 51638 message, full-sensor rosbag containing LiDAR, RGB-D,
IMU, GNSS, optical flow, TF, and LIO outputs. It is not redundant cache and is
retained for deterministic replay.

## Runtime reductions

The GPS/optical-flow ExternalNav entrypoint now disables the D435 and MID360 ROS
conversion bridges by default. Their Gazebo sensors remain in the model, so the
full sensor launch can restore both bridges with:

```bash
ENABLE_D435_BRIDGE=1 ENABLE_MID360_BRIDGE=1 \
  bash tools/run_sim_with_externalnav.sh
```

The ExternalNav launch starts only the GNSS and optical-flow fault injectors.
The full `sensor_pipeline.launch.py` remains unchanged for multi-sensor fault
campaigns. An unreachable duplicate optical-flow startup block was removed from
the stack script.

For routine GPS/flow work, record the lightweight rosbag profile:

```bash
UF_BAG_PROFILE=nav \
  bash src/ultra_fusion_nav/scripts/record_sensor_bag.sh
```

Use `UF_BAG_PROFILE=full` for milestone captures that need raw LiDAR and RGB-D.

## Retention

Preview safe cleanup candidates:

```bash
bash tools/prune_runtime_cache.sh --keep-runs 8
```

Apply the previewed cleanup with `--apply`. The tool does not remove milestone
directories, rosbags, JSON reports, or directories containing a `.keep` marker.

## Runtime verification

The cleanup removed about 40 MB of regenerable colcon and old temporary run logs.
The 2.884 GB milestone rosbag and all JSON reports were verified after cleanup.
The stack shutdown path was also corrected to use `SIGINT`, `SIGTERM`, then
`SIGKILL`; two shutdown smoke tests and all subsequent flights left no Gazebo or
ArduPilot process and no stale lock.

Three independent 6 m by 4 m rectangle flights used the lightweight profile.
All completed takeoff, four sides, four turns, return, and landing with
`ekf_using_gps=False`. Summary statistics were:

- ATE RMSE mean 0.111 m, range 0.092-0.126 m;
- 1 s RPE RMSE mean 0.100 m, range 0.087-0.111 m;
- RTF median mean 0.9999;
- optical flow mean 14.64 Hz, minimum 13.91 Hz;
- GNSS mean 9.64 Hz;
- ExternalNav mean 19.56 Hz, minimum 18.69 Hz.

Exact run identifiers and values are in `externalnav_lightweight_trials.csv`.
