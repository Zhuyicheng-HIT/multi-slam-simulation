
# Visual SLAM to Ultra-Fusion integration

## Supported boundary

The current integration targets the official binary-only Ultra-Fusion ROS 2
Humble v0.2.2 runtime. The confirmed visual path is raw D435i RGB-D into the
Ultra-Fusion internal visual frontend; no external feature-track message is
required or invented.

```text
D435i RGB-D в”Ђв”Ђв”¬в”Ђв”Ђ raw RGB -> compressed-image bridge -> Ultra-Fusion frontend
              в”‚                                      -> local visual factors
              в””в”Ђв”Ђ RTAB-Map -> loop/relocalization/map->odom global correction

MID360 PointCloud2 + D435i IMU ---------------------> Ultra-Fusion LVIO
```

The D435i point cloud and `/rtabmap/cloud_map` are not inputs. MID360 raw
PointCloud2 provides LiDAR geometry.

## Preparing a complete profile

Do not author a reduced YAML. Start from the complete official ROS 2 LVWIO
profile installed by v0.2.2 or extracted from its verified Debian package:

```bash
python3 tools/prepare_ultrafusion_d435i_config.py \
  /opt/ultrafusion/config/m3dgr/uf_m3dgr_ros2_lvwio.yaml \
  /tmp/ultrafusion_d435i
```

The generator preserves all upstream fields and changes only the confirmed
sensor mode, topics, camera file, frequencies, disabled map export, and
simulation extrinsics. It selects:

- native LVIO: IMU + LiDAR + image, wheel disabled;
- `/front/d435i/imu` as estimator body IMU;
- `/sim/mid360/points_raw` as LiDAR;
- a compressed derivative of the D435i RGB stream;
- aligned `16UC1` depth;
- fixed simulation extrinsics and zero simulated camera delay.

The generated extrinsics are valid only for
`models/iris_apm_rgbd/model.sdf`. A physical aircraft must use a measured
CameraвЂ“IMU/LiDAR calibration and should begin with online calibration disabled.

## Running the bridge

With the simulation stack already running and the official binary installed:

```bash
tools/run_ultrafusion_d435i_bridge.sh \
  --config /tmp/ultrafusion_d435i/ultrafusion_d435i_mid360_lvio.yaml
```

Use `--dry-run` to validate live topic types and depth encoding without starting
the binary. The wrapper checks RGB, depth, D435i IMU, MID360 PointCloud2 and
visual reliability, then starts the bounded raw-to-compressed image transport.
Its trap signals only the two exact process groups it created.

The official runtime was deliberately not installed during this task. A real
LVIO replay remains conditional on installing the verified v0.2.2 package in
the supported Humble runtime and confirming how its `preprocess.lidar_type: 7`
interprets this simulator's PointCloud2 `time` field.

## Recording and checking a baseline

With one complete stack active:

```bash
tools/record_ultrafusion_visual_inputs.sh \
  --duration 90 \
  --output artifacts/.../bags/ultrafusion_visual_input_baseline

python3 tools/check_ultrafusion_visual_inputs.py \
  artifacts/.../bags/ultrafusion_visual_input_baseline
```

The recorder discovers only an exact allow-list. It excludes D435i point cloud,
RTAB colored cloud and FAST-LIO registered visualization cloud while retaining
MID360 raw geometry. The checker reads the bag offline, verifies counts,
frequencies, monotonic stamps, RGB/depth pairing, CameraInfo, frames, TF, depth
encoding and reliability, and writes JSON plus Markdown with a meaningful exit
code.

## Reliability and duplicate-counting policy

The current `/vision/reliability_*` topics are valuable side-channel evidence,
but v0.2.2 exposes no public subscriber for them. Connecting those scores to
the internal factor scheduler is therefore future upstream work, not a remap.

RTAB-Map odometry and Ultra-Fusion visual tracks come from the same RGB-D
measurements. Do not inject both as independent high-weight local factors.
If RTAB is retained, restrict it to low-frequency global information:

- GlobalClosure and LocalSpaceClosure events;
- cross-session relocalization;
- `mapв†’odom` global correction;
- explicitly covariance-inflated constraints after correlation analysis.

The next engineering step is direct LVIO replay against the official binary,
not a new KLT frontend. A separate feature frontend becomes justified only if a
future upstream release publishes an external feature-track contract.
