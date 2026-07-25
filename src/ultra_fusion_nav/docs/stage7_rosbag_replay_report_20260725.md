# Stage 7 ROS Bag Replay Report

Date: 2026-07-25

## Input

The replay uses the repository-local bag:

`logs/uf_stage2_20260713_102507/bag/sensors`

It is 224.02 s, 2.8 GiB, and contains 51,638 messages. The extractor reads
`/lio/odom`, `/Odometry` (evaluation reference only),
`/sensors/gnss/fix`, `/sensors/optical_flow/rad`, and `/lio/diagnostics`.

## Compatibility findings

The bag's `LioDiagnostics.msg` predates the current message fields, so all 1,098
diagnostic messages are intentionally skipped rather than deserialized with
fabricated fields. The factor JSON records
`lidar_diagnostics_available=false` and uses an explicitly marked approximate
LIO decision. New bags must be recorded after the current interface build.

The optical-flow header clock is relative (`208.428` to `318.219 s`) while LIO
and GNSS headers are absolute (`1.7839e9 s`). The extractor found a plausible
clock offset of `1783909743.137 s` from the overlapping recording start and
preserved it in `flow_stamp_offset_s`. It produced 528 flow intervals. Of
these, 301 fail the hard MTF01P quality gate (`quality < 20`) and are disabled;
the remaining 227 use the continuous reliability weight. This prevents a
zero-quality sample from becoming a backend constraint merely because its
displacement agrees with the prediction.

## Replay result

The backend was run as a true 20-state rolling window. It produced 1,098 state
updates, 40 active factors in the final window, and fixed/dynamic output files
under `logs/uf_stage7_rosbag_20260725/replay_window20/`.

| Variant | RMSE vs `/Odometry` reference* | GNSS residual p50 | GNSS residual p95 | Flow residual p50 | Normal-equation cost |
|---|---:|---:|---:|---:|---:|
| fixed_weight | 0.01269 m | 0.40368 m | 0.45774 m | 0.03226 m | 3473.49 |
| scheduler_weighted | 0.01450 m | 0.39875 m | 0.45464 m | 0.02998 m | 1473.62 |

\* `/Odometry` and `/lio/odom` are from the same FAST-LIO chain; this is not an
independent ground-truth ATE/RPE result. The dynamic run improves factor
consistency and cost, but this bag cannot prove trajectory accuracy superiority.

## Reproduction commands

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run uf_backend_fusion extract_backend_factors \
  --bag logs/uf_stage2_20260713_102507/bag/sensors \
  --output logs/uf_stage7_rosbag_20260725/factors.json
ros2 run uf_backend_fusion replay_backend_factors \
  --factors logs/uf_stage7_rosbag_20260725/factors.json \
  --output-dir logs/uf_stage7_rosbag_20260725/replay_window20 \
  --window-size 20
```

## Next gate

Record a new same-version bag with `/reliability/*_score` and
`/reliability/scheduler_state`, preserve an independent Gazebo truth topic for
evaluation only, and add real IMU preintegration factors. Until then this is a
validated factor-extraction/replay pipeline, not final Ultra-Fusion accuracy
evidence.
