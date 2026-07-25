# Public Dataset Manifest

Raw public data stays on `E:` and is not copied into the Git worktree. This
iteration keeps one small, usable public-data input and does not start another
large download.

## Selected reproducible input

| Field | Value |
| --- | --- |
| Dataset | M3DGR |
| Sequence | Corridor01, recovered complete-chunk prefix |
| Official source | https://github.com/sjtuyinjie/M3DGR |
| Full sequence advertised size | 6.39 GB, 403 s |
| Download snapshot | `Corridor01_prefix_download.bag`, 1,042,747,392 bytes |
| Usable recovered bag | `Corridor01_prefix_recovered.bag`, 1,042,131,970 bytes |
| Recovered duration | 63.195 s |
| Topics | 9 topics, 45,379 messages |
| E drive raw path | `/mnt/e/ultra-fusion-datasets/m3dgr/raw/Corridor01_prefix_recovered.bag` |
| ROS 2 output | `/mnt/e/ultra-fusion-datasets/ros2/Corridor01_prefix_humble` |
| Metadata report | `/mnt/e/ultra-fusion-datasets/reports/m3dgr_corridor01_prefix.json` |

The recovered bag is a local prefix reconstruction, not an official full-file
checksum artifact. It contains only complete ROS1 compressed chunks from the
download snapshot; the incomplete tail was omitted and a fresh index was
written. The original snapshot is retained beside it for auditability.

## Conversion and inspection

```bash
source /opt/ros/humble/setup.bash
python3 src/ultra_fusion_nav/scripts/inspect_public_bag.py \
  --input /mnt/e/ultra-fusion-datasets/m3dgr/raw/Corridor01_prefix_recovered.bag \
  --output /mnt/e/ultra-fusion-datasets/reports/m3dgr_corridor01_prefix.json

/home/zyc/.local/bin/rosbags-convert \
  --dst-version 8 \
  --src-typestore ros1_noetic \
  --dst-typestore ros2_humble \
  --src /mnt/e/ultra-fusion-datasets/m3dgr/raw/Corridor01_prefix_recovered.bag \
  --dst /mnt/e/ultra-fusion-datasets/ros2/Corridor01_prefix_humble
```

Conversion is an interoperability step, not an estimator result. Preserve the
recorded timestamps and never feed `/odom`, Gazebo truth, or FCU fused position
back into the tested estimator.

## Paper-aligned evaluation slices

The first offline slice is fixed at the recovered 63.195 s prefix. The
acceptance record will include:

1. raw topic counts and timestamp monotonicity;
2. LiDAR/IMU overlap and LiDAR acquisition-time handling;
3. static/dynamic point filtering and local-map protection;
4. LIO ATE/RPE plus point-to-plane/Hessian diagnostics;
5. `D_L`, `D_I`, and visual evidence coverage before scheduler weighting.

Ultra-Fusion equation (15) remains reserved for Stage 6 factor enable/disable
and hysteresis. Equations (18)-(23) are used for diagnostic evidence mapping;
missing evidence stays unavailable rather than being synthesized.

## Retained incomplete downloads

The browser-created MARS-LVIG `HKairport03.bag` partial and the original
M3DGR browser partial remain in the Windows Downloads directory. They are not
used as bags, are not deleted, and are not included in the current evaluation.
