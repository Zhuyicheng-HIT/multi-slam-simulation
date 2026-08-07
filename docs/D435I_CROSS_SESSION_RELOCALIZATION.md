
# D435i RTAB-Map cross-session relocalization baseline

## Scope and definition

This experiment is a true process-boundary test of the existing D435i RGB-D
RTAB-Map baseline. Session 1 is stopped completely. Session 2 starts a new
Gazebo/SITL/MAVROS/D435i stack and new RTAB-Map processes. The only retained
state is an on-disk Session 1 RTAB-Map database.

This is not same-process loop closure, continued mapping, retained TF state, or
FAST-LIVO2-assisted localization. RTAB-Map visual odometry remains the internal
odometry source.

## Database and localization contract

Session 1 uses `d435i_rtabmap_feature_aligned.yaml` and a fixed 4.50 m
`long_loop_return` at 0.20 m/s, 0.50 m flight height and yaw 0. The reference is
accepted only when the database is readable, contains nodes and visual words,
contains a GlobalClosure link, has a live closure event with at least 10
geometry inliers, and reports lost/reset 0/0.

The accepted database is copied to a mode-0444 mother file. Every Session 2
copies that mother to its own writable database before RTAB-Map starts. Mother
SHA-256 is checked before and after every attempt.

Session 2 uses `d435i_rtabmap_localization.yaml`:

- feature detector, feature type, odometry and `Vis/MinInliers=10` match the
  feature-aligned mapping baseline;
- `Mem/IncrementalMemory=false` enables localization mode;
- `Mem/InitWMWithAllNodes=true` loads the small reference graph into working
  memory;
- `Mem/LocalizationDataSaved=false` avoids treating Session 2 as continued
  mapping;
- `delete_db_on_start=false` preserves the per-attempt database copy;
- RGB/depth exact synchronization and RTAB-Map visual odometry remain enabled.

`Mem/LocalizationReadOnly` is deliberately false on the disposable child copy.
The immutable object is the mother database, not RTAB-Map's runtime SQLite
handle.

## Frozen conditions

The machine-readable definitions are in
`config/d435i_relocalization_conditions.yaml`. They were frozen before the
formal matrix. Coordinates are MAVROS local offsets from the common Gazebo
spawn point. The D435i is front-facing: camera +X is vehicle +X at yaw 0.

The textured world is open around the reference corridor. All targets are
inside the already validated safety box and remain far from the walls and
obstacles. The reference maps the center line from x=0 to x=4.50 m at y=0.

| Condition | x (m) | y (m) | z (m) | yaw | Purpose |
|---|---:|---:|---:|---:|---|
| `start_same` | 0.00 | 0.00 | 0.50 | 0Â° | mapped start hover, same view |
| `start_reverse` | 0.00 | 0.00 | 0.50 | 180Â° | same position, large view change |
| `route_middle` | 2.25 | 0.00 | 0.50 | 0Â° | mapped route midpoint |
| `route_end` | 4.25 | 0.00 | 0.50 | 0Â° | near mapped endpoint |
| `mapped_edge` | 3.75 | 0.45 | 0.50 | 0Â° | near mapped corridor edge |
| `similar_geometry` | 2.25 | 0.60 | 0.50 | 0Â° | repetitive grid, lateral geometric offset |

Each target performs a bounded yaw sweep of 12Â° or 18Â°, then holds the original
view. RTAB-Map starts only after the aircraft reaches the target, so Session 2
does not map or visually odometer-integrate the positioning flight.

## Success and evidence

An event is geometry-accepted only when RTAB-Map reports an accepted global or
proximity match with at least 10 visual inliers. An attempt is a successful
relocalization only when all of the following also hold:

- the accepted node is within 1.25 m of the GT-relative mapped position;
- map-aligned pose reaches position error at most 0.75 m and yaw error at most
  45Â° for five consecutive samples;
- no accepted visually similar but geometrically wrong candidate is present;
- there is no post-alignment position jump above 1 m;
- lost/reset and TF backward-jump counts are zero;
- the child database is readable and the mother hash is unchanged;
- the flight, landing, process cleanup, active-marker cleanup and port audit
  complete.

The monitor records candidate IDs, matched IDs, map IDs, posterior/likelihood,
visual matches, visual words, geometry inliers, closure transform, map-to-odom,
RTAB odometry, GT, TF and event timing. A GlobalClosure log line alone is not a
success criterion.

## Smoke result

Experiment `cross_session_v1_smoke_20260730_01` passed:

- Session 1: 160 nodes, 3915 words, 34 GlobalClosure database links, 20 live
  geometry-validated closure events, maximum 70 inliers, lost/reset 0/0;
- Session 2 `start_same`: matched node 133 in map 0 with 76 geometry inliers
  and 214 visual words;
- stable alignment latency 0.178 s from the first RTAB-Map Info event;
- stable position/yaw error 0.066 m / 0.064Â°;
- maximum map-to-odom translation jump 0.050 m;
- abnormal post-alignment jumps, lost, reset and TF backward jumps all zero;
- the reference mother SHA-256 remained unchanged and both sessions passed
  active/PID/port cleanup.

## Running

After building and sourcing the workspace, a smoke with an independent
reference map is:

```bash
MATRIX_ID=cross_session_v1_smoke \
CROSS_SESSION_CONDITIONS=start_same \
VALID_RUNS_PER_CONDITION=1 \
MAX_ATTEMPTS_PER_CONDITION=3 \
REQUIRE_SUCCESS=1 \
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_cross_session_matrix.sh
```

The formal matrix uses the same script with all six default conditions and
three valid attempts per condition. It can reuse an already validated mother
by passing `REFERENCE_DB` and `REFERENCE_METADATA`; each Session 2 still gets a
fresh child copy.
