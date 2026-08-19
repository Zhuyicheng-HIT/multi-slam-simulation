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
/fusion/unified/odom --------------------------+
```

The observer waits in a bounded queue for bracketing committed poses, applies
the per-point `offset_time`, and transforms each point with `T_world_body *
T_body_lidar`. This is deliberately delayed and suitable for observer A/B only.
It is **not** yet the future pre-FAST-LIO deskew contract: that path must use the
same causal IMU prediction trajectory as FAST-LIO and must not depend on a pose
that already consumed the scan.

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
voxel that was repeatedly observed as free before the current scan. Endpoint
guards, delayed free confirmation, unknown-space output, bounded neighborhood
growth, and slow occupied recovery protect static structure and pose drift.

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

The benchmark runs all ten scenarios three times, compares the conservative
observer to the existing `TemporalVoxelFilter` contract, and reports detection,
static protection, map contamination/completeness, P50/P95 latency, and peak RSS.
ATE/RPE/residual deltas are exactly zero by construction because neither branch
is connected to FAST-LIO; they are not claims about closed-loop improvement.
