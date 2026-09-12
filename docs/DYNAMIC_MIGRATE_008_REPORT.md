# DYNAMIC-MIGRATE-008

## Migration

Migrated the validated PR15 Dynamic Observer v2 and Clean Scan Gateway into
the HXY baseline as independent packages:

- `uf_dynamic_interfaces/PreviousFastLioState`
- `uf_dynamic_observer` causal IMU deskew, visibility-aware Dynamic Observer
  v2, clean admission, fail-open gateway, launch/configuration, benchmark and
  evaluator tests
- FAST-LIO wrapper support for an explicit clean input topic and previous-state
  export

The runtime chain is:

`raw MID360 /livox/lidar -> Dynamic Observer/Clean Gateway -> clean topic -> FAST-LIO -> NativeLidarFactor -> HXY/backend`.

Raw `/livox/lidar` remains published for safety/avoidance and is never
remapped in place. The gateway uses the same independent MID360 `/livox/imu`
stream and the causal FAST-LIO previous-state export; if that contract is not
available it fails open and does not silently fabricate a clean scan.

## Dynamic benchmark

The migrated PR15 benchmark completed successfully on 18 deterministic
scenarios, 3 seeds and 2 repeats:

| metric | migrated v2 |
|---|---:|
| micro precision | 99.8439% |
| micro recall | 97.0854% |
| micro F1 | 98.4454% |
| macro precision | 93.5449% |
| macro recall | 85.7748% |
| macro F1 | 88.8424% |
| static preservation | 99.9859% |
| dynamic-as-static contamination | 1.8083% |
| latency P50/P95 | 7.19 / 9.78 ms |

The micro result exceeds the old PR15 reference (98.25 / 76.16 / 85.81),
while macro recall exposes the known hard scenarios (small fast target,
opening/closing door, occlusion appear/disappear and far sparse target). No
recall-expanding deletion heuristic was added.

## Tests and integration status

`uf_dynamic_observer` builds with the local MID360 overlay and its tests pass:
21 C++ tests, 3 evaluator-contract tests, 40 total CTest/pytest results. The
FAST-LIO wrapper shell syntax and clean launch Python also pass basic checks.

A 60-second static Gazebo run was attempted after building all missing current
workspace packages. The stack reached `/fusion/unified/diagnostics`, but the
sensor supervisor then failed its `/clock` validity gate before the estimator
recorders could collect a complete 60-second trajectory. Consequently there
is no honest 10/30/60-second FAST-LIO/backend/unified-odom drift statistic from
this run; no static drift conclusion is claimed. The failure is environmental
startup (`ROS /clock did not produce a valid Gazebo simulation timestamp`), not
a Dynamic Observer transaction or estimator drift.

No Z, state machine, relocalization, Dynamic V2 algorithm thresholds,
prediction recovery, scan contract, or HXY mathematics were changed.
