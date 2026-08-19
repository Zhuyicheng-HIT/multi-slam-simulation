# UF dynamic observer

This ROS 2 Humble package is an opt-in, side-channel prototype for class-agnostic
dynamic-point removal. It does **not** remap, republish, or modify the FAST-LIO
input. The current five-source backend and one-observation-one-factor contract
therefore remain unchanged.

## Data contract

Default input is the exact Livox protocol topic already consumed by FAST-LIO:

```text
/livox/lidar (livox_ros_driver2/CustomMsg) ----+----> FAST-LIO (unchanged)
                                               |
                                               +----> uf_dynamic_observer
previous /Odometry ----------------------------+
/livox/imu ------------------------------------+
```

The observer waits in a bounded queue, selects the latest FAST-LIO posterior no
newer than scan start, and propagates raw IMU to every Livox nanosecond
`offset_time`. A bounded terminal zero-order hold is permitted only inside the
configured IMU gap. Future pose/IMU samples, timestamp regressions, and larger
gaps are rejected. No unified pose is consumed, so the contract does not form a
future unified-backend feedback cycle.

Outputs are in the configured world frame:

- `/dynamic_observer/static_candidates`
- `/dynamic_observer/dynamic_candidates`
- `/dynamic_observer/unknown_candidates`
- `/dynamic_observer/scored_cloud` (`dynamic_score` field)
- `/dynamic_observer/statistics`
- `/dynamic_observer/latency_diagnostics`

No TF is published. Truth labels are never subscribed by the node.

## Algorithm scope

The clean-room prototype borrows the conservative visibility principle shared by
FreeDOM and DUFOMap: a point is a strong dynamic candidate only when it enters a
voxel that was repeatedly observed as free before the current scan. It also
tracks measured vacated surfaces, occlusion-safe static evidence, bounded
dynamic hold, and range-adaptive static dwell. Endpoint guards, unknown-space
output, bounded neighborhood growth, and slow occupied recovery protect static
structure and pose drift.

It is not a verbatim port of FreeDOM. In particular, MID360 angular inpainting /
raycast enhancement remains disabled until a measured non-repetitive scan-pattern
mask can be validated; inventing a spinning-LiDAR range image would create false
free space.

## Run

The launch defaults to disabled:

```bash
ros2 launch uf_dynamic_observer observer.launch.py
```

Enable observer mode without changing FAST-LIO ownership:

```bash
ros2 launch uf_dynamic_observer observer.launch.py enabled:=true
```

For the legacy simulator PointCloud2 path:

```bash
ros2 launch uf_dynamic_observer observer.launch.py \
  enabled:=true input_mode:=pointcloud2
```

Before hardware use, set the audited MID360 `T_body_lidar` in `observer.yaml`.

## Deterministic A/B

```bash
ros2 run uf_dynamic_observer dynamic_observer_benchmark /tmp/dynamic_observer.json
```

The benchmark runs 18 low-altitude scenarios with three deterministic seeds and
two repeats per seed. It compares TemporalVoxelFilter, frozen observer v1, and
visibility-aware observer v2, reporting per-scenario detection, static
protection, contamination/completeness, unknown ratio, P50/P95/P99 latency,
thread CPU, and filter memory.
ATE/RPE/residual deltas are exactly zero by construction because neither branch
is connected to FAST-LIO; they are not claims about closed-loop improvement.

See `docs/DYNAMIC_OBSERVER_V2_VALIDATION.md` and
`docs/DYNAMIC_OBSERVER_V2_ARCHITECTURE.md` for the final matrix and gate.
