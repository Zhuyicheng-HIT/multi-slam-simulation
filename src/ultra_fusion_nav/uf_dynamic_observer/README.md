# UF dynamic observer

This ROS 2 Humble package contains an opt-in, side-channel class-agnostic
dynamic observer and a separate, default-off Clean Scan Gateway candidate. The
original `/livox/lidar`, production FAST-LIO, five-source backend, ExternalNav,
and one-observation-one-factor contract remain unchanged.

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

## Clean Scan Gateway candidate

The gateway publishes a new namespaced Livox `CustomMsg`; it never republishes
or remaps `/livox/lidar`. STATIC and UNKNOWN points are retained and only
DYNAMIC_CONFIRMED points are removed. Every retained point preserves its
coordinates, `offset_time`, `line`, `tag`, and reflectivity; the message keeps
its `header`, `timebase`, `lidar_id`, and reserved fields.

The state handoff is one-way and causal:

```text
raw scan i -> gateway -> Clean FAST-LIO -> posterior i
                  ^                |
                  +-- posterior i-1+
                  +-- IMU <= each point time
```

The bounded queue waits for the most recent completed Clean FAST-LIO posterior
strictly preceding the scan. It never reads Raw FAST-LIO, unified pose, the
current scan posterior, or future IMU. Missing/stale state, IMU coverage
failure, timestamp regression, queue overflow, latency, malformed
classification, or internal exception produces an exact raw passthrough and an
explicit degraded/fail-open diagnostic. Gateway failure cannot drop a scan or
emit an empty frame.

The launch remains disabled by default:

```bash
ros2 launch uf_dynamic_observer clean_gateway.launch.py
ros2 launch uf_dynamic_observer clean_gateway.launch.py enabled:=true
```

The enabled form is only for an independently namespaced Clean FAST-LIO A/B.
It does not authorize production LiDAR cutover. See
`docs/DYN_INTEGRATION_005_CLEAN_GATEWAY.md` for the frozen replay and gate.

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

## Opt-in long-term static map refinement

`long_term_static_map_node` is a project-owned, downstream map product. It is
disabled by default, never remaps `/livox/lidar`, never publishes TF, and is
not an estimator input. Missing previous-state evidence holds the last valid
map and publishes a degraded status; it cannot drop a LiDAR scan.

```text
observer v2 scored cloud + strictly previous FAST-LIO posterior
                    |
                    v
 UNKNOWN <-> STATIC_CANDIDATE <-> STATIC_CONFIRMED
    ^              |                    |
    |              v                    v
    +---- DYNAMIC_CANDIDATE <-> DYNAMIC_CONFIRMED
```

Only `STATIC_CONFIRMED` is published on:

- `/mapping/long_term_static/points`
- `/mapping/long_term_static/relocalization_points`
- `/mapping/long_term_static/loop_closure_points`

Promotion requires repeated occupied support, time persistence and multiple
measured viewpoints. Demotion requires actual free-ray traversal; absence of a
MID360 return never means free space. Far sparse returns use a deliberately
longer admission history and otherwise remain `UNKNOWN`.

```bash
ros2 launch uf_dynamic_observer long_term_static_map.launch.py
ros2 launch uf_dynamic_observer long_term_static_map.launch.py enabled:=true
```

To test the purified keyframe data path, start relocalization with both its
normal configuration and `config/relocalization_static_admission.yaml`. This is
an explicit opt-in overlay; the existing `/lio/local_map` default is preserved.
The analogous `config/shared_mapping_static_admission.yaml` overlay makes the
long-lived shared-map product consume only this confirmed snapshot; it does not
alter the online source-aware map unless explicitly selected.

The optional `/semantic/dynamic_evidence` PointCloud2 input expects fields
`x/y/z/dynamic_confidence`. It is disabled by default and defaults to shadow
mode when enabled. Geometry remains class-agnostic and fully functional when
no camera semantic evidence is present.

See `docs/DYN_MAP_006_LONG_TERM_STATIC_REFINEMENT.md` for the lifecycle,
three-map validation matrix, runtime cost, and production blockers.

The benchmark runs 18 low-altitude scenarios with three deterministic seeds and
two repeats per seed. It compares TemporalVoxelFilter, frozen observer v1, and
visibility-aware observer v2, reporting per-scenario detection, static
protection, contamination/completeness, unknown ratio, P50/P95/P99 latency,
thread CPU, and filter memory.
ATE/RPE/residual deltas are exactly zero by construction because neither branch
is connected to FAST-LIO; they are not claims about closed-loop improvement.

See `docs/DYNAMIC_OBSERVER_V2_VALIDATION.md` and
`docs/DYNAMIC_OBSERVER_V2_ARCHITECTURE.md` for the final matrix and gate.
