# Joint-map Long-run Causal Analysis

## Experiment

All prescribed runs used the same 10 m × 6 m rectangle, 0.5 m/s command,
balanced visual cadence, unchanged FRS/integrity thresholds and the software
rendering fallback.  The first matrix contains three consecutive runs per mode.
A fourth run per mode was retained after FCU StatusText gained direct wall
monotonic timestamps; it is supplementary evidence, not a replacement for a
failed run.

| Mode | Prescribed runs | Supplemental r4 | Prescribed rollback totals |
|---|---|---|---|
| map OFF | 0/3 LAND; 3 FCU Crash | FCU Crash | 0, 6, 1 |
| LiDAR-only | 0/3 LAND; 3 FCU Crash | LAND/disarm PASS | 8, 0, 0 |
| RGB-D + LiDAR joint | 0/3 LAND; 3 FCU Crash | FCU Crash | 48, 15, 6 (cycle trace) |

Every failed flight reported ArduPilot's `Crash: Disarming` with attitude error
above 30 degrees and very low acceleration, normally during the second turn or
second edge.  The flight controller reported `navigation_source=gps`; the
unified estimator was not the FCU control source in these experiments.

## Event ordering

The original nine runs predate direct monotonic FCU logging.  Their accumulated
data still include decisive samples: map-off r1 and LiDAR-only r2/r3 crashed
with zero rollback over the complete recorded run.  Thus estimator rollback and
mapping are not necessary causes of the FCU failure.

The direct-clock supplemental runs remove wall-clock reconstruction ambiguity:

- **joint r4:** FCU Crash at monotonic 17474.580566 s; first rollback 1.736 s
  later.  Crash-before-rollback is exact.  There were zero rollback events
  before Crash.
- **map-off r4:** 13 rollback events preceded Crash and 21 followed it, but no
  mapping process was enabled and the FCU remained on GPS navigation.  This is
  consistent with both estimators reacting to the same simulated vehicle
  dynamic failure, not a map-to-FCU causal path.
- **LiDAR-only r4:** completed all four edges, LAND and disarm with 884 Native
  LiDAR factors, 0 optimization errors, 0 integrity rejects and 0 rollback.

## Mapping load versus backend timing

The exact-clock runs show no stable
`map spike → solver spike → state gap → excessive correction → FCU Crash`
sequence.

| Metric before Crash/end | map OFF r4 | LiDAR-only r4 | joint r4 |
|---|---:|---:|---:|
| solver P50 / P95 | 37.13 / 76.23 ms | 41.18 / 77.08 ms | 39.73 / 76.23 ms |
| max backend ROS-state gap | 0.297 s | 0.594 s | 0.331 s |
| map publish P95 | n/a | 139.86 ms | 49.90 ms |
| LiDAR insertion P95 | n/a | 9.54 ms | 9.19 ms |
| RGB-D insertion P95 | n/a | n/a | 109.50 ms |
| rollback before Crash/end | 13 | 0 | 0 |
| flight result | FCU Crash | LAND/disarm | FCU Crash |

The successful LiDAR-only run tolerated larger full-map publish spikes than the
failed joint run.  The joint run had no pre-Crash rollback and no >1 s state
gap.  Therefore no map performance isolation change was justified.  The
existing sensor-data QoS and RGB/depth caches are bounded; the diagnostic data
does not show unbounded application-level growth.

## Map result

Across four joint runs, voxel count was 54,856–97,544 (median 88,473), RGB
coverage 16.97–26.61% (median 21.31%), and supplementary occupied volume
28.72–57.12% (median 38.45%).  Conflict ratio and conflict-derived ghosting
proxy were 0 in all four runs, and there were zero evictions.  LiDAR remained
the primary geometry source; RGB-D added color and non-conflicting occupancy.

LiDAR-only r4 reached 91,664 voxels during the complete route.  The shorter
failed LiDAR-only runs reached 47,030–47,623 voxels, explaining the larger r4
map without a semantic change.

## FCU conclusion

The old 27-rollbacks observation combined two independent effects:

1. ArduPilot/Gazebo intermittently loses vehicle attitude/altitude during the
   turn and declares Crash, including with mapping disabled and zero rollback.
2. After a crash, invalid vehicle motion and sensor data can provoke legitimate
   integrity rejection and transaction rollback in the unified backend.

The single retained successful long route under identical settings proves the
failure is not deterministic, but 1/12 overall route success is not sufficient
for tethered-flight authorization.  WSL software rendering and the simulated
FCU/dynamics path remain the limiting environment.
