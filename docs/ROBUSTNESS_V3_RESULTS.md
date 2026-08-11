# Ultra-Fusion Robustness V3 validation results

## Scope and evidence contract

This report freezes Performance V2 at
`d04f88d422de611454cf9454ffa2cc3a5741dab3` and evaluates Robustness V3 from
outside the estimator.  No fusion factor, physical sensor model, integrity
threshold, rollback rule, D_V/FRS threshold, or `/Odometry` fallback was
changed.  The levels in the profile YAML are explicit engineering test points;
they are not claimed as limits from the Ultra-Fusion paper.

The campaign contains 88 deterministic replay reports: 38 single-fault, 40
time/extrinsic-calibration, 8 double-fault, and 2 endurance runs.  Every replay
used the same immutable sensor payloads and the same frozen nominal backend
trajectory as its alignment reference.  Therefore replay ATE/RPE below are
**delta-to-frozen-nominal**, not absolute simulator-ground-truth accuracy.
All 88 replays had zero optimization error, zero true integrity rejection, zero
transaction rollback, and zero `/Odometry` fallback.

## FRS ON/OFF result

Among paired runs with valid aligned trajectories, FRS ON reduced delta-ATE in
36 cases and increased it in 5.  The median benefit was 0.001188 m.  This shows
that FRS is broadly protective, but it is not uniformly beneficial: LiDAR
medium degradation and IMU medium bias are counterexamples and must remain in
the regression set.

Median fault-response times were 0.070 s for vision, 0.177 s for LiDAR,
0.009 s for GNSS, 0.018 s for optical flow, and 0.109 s for IMU.  Median
measured recovery times were 0.047 s for vision, 0.496 s for LiDAR, 0.471 s for
flow, and 0.008 s for IMU.  GNSS recovery cannot be asserted from this replay:
the frozen capture does not contain live FCU GNSS innovation metadata, so the
fresh scheduler correctly holds GNSS invalid at zero weight.  The injected
GNSS jump A/B still validates isolation behavior, but detection and recovery
must be repeated with live FCU innovation evidence.

Representative A/B results:

| Profile | FRS ON delta-ATE | FRS OFF delta-ATE | Completeness | Finding |
|---|---:|---:|---:|---|
| nominal | 0.002278 m | 0.003623 m | 1.000 | reference |
| visual heavy, 6 s dropout | 0.003669 m | 0.004603 m | 1.000 | continuous |
| LiDAR medium, 60% correspondence dropout | 0.018342 m | 0.017146 m | 1.000 | continuous; ON is slightly less accurate |
| GNSS jump heavy, 35 m | 0.002307 m | 0.012175 m | 1.000 | strongest measured FRS protection |
| flow heavy, 25 s outage | 0.002281 m | 0.003625 m | 1.000 | continuous |
| IMU medium, 0.05 rad/s and 0.30 m/s2 bias | 0.005890 m | 0.003646 m | 1.000 | continuous; ON is less accurate |
| LiDAR heavy, 25 s outage | approximately 0 m | 0.000068 m | 0.512 | failed continuity in both modes |
| IMU heavy, 25 s outage | approximately 0 m | 0.000069 m | 0.416 | failed continuity in both modes |

Near-zero ATE on an incomplete heavy-outage trajectory is not a success: only
the short surviving prefix aligned.  Completeness and maximum odometry gap are
the governing criteria for those cases.

## Measured boundaries

These are only the pass/fail bounds exercised by this campaign, not exhaustive
physical limits:

* Vision remained continuous through the tested 6 s heavy outage.
* Native LiDAR remained continuous with 60% correspondence dropout; a 25 s
  outage failed at 0.512 route completeness.
* GNSS denial for 30 s and a 35 m position jump remained continuous, subject to
  the frozen-innovation limitation above.
* Optical-flow outage for 25 s remained continuous.
* IMU bias of 0.05 rad/s plus 0.30 m/s2 remained continuous; a 25 s outage
  failed at 0.416 completeness.
* Camera-to-IMU offsets passed at +100 ms and -50 ms, the largest tested
  positive/negative values.  The actual limit lies outside or between untested
  points and is not inferred.
* Native LiDAR timing was the most dangerous tested fault.  Even +2 ms produced
  a trajectory gap greater than 1 s; +/-5 ms and +20 ms commonly left only one
  usable native factor.  There is no demonstrated nonzero tolerant operating
  interval in this frozen replay.
* D435i extrinsics passed at the largest tested errors: 8 degrees rotation and
  15 cm translation.
* MID360 passed at 3 degrees rotation; 8 degrees failed with 0.77
  completeness.  A 15 cm translation passed the broad continuity criterion,
  but delta-ATE increased to 0.0856 m versus 0.0285 m at 5 cm, so it is not a
  recommended calibration allowance.

## Double faults and endurance

Visual+GNSS medium, LiDAR+GNSS medium, and IMU+flow medium retained complete
trajectories with zero optimization/integrity/rollback errors in both FRS
modes.  Visual+LiDAR heavy failed in both modes at 0.512 completeness.  FRS ON
improved visual+GNSS, while the LiDAR+GNSS and IMU+flow pairs were slightly
more accurate with FRS OFF; this is why the result is a measured boundary, not
a blanket robustness claim.

The long cyclic visual/GNSS replay remained complete.  FRS ON produced delta
ATE 0.002825 m, RPE 0.003716 m / 0.015354 deg, 26.888 ms median solver time,
58% process CPU, and 91,516 KiB peak RSS.  FRS OFF produced 0.003350 m,
0.004029 m / 0.019241 deg, 32.211 ms, 62% CPU, and 93,480 KiB.  Both had zero
optimization/integrity/rollback errors.

## Online joint-map stress result

The final full-stack run verified the corrected launch contract:
`paper_reprojection` was active, the mapper consumed
`/cloud_registered_filtered`, NativeLidarFactor was used, and no pose fallback
was enabled.  Before the vehicle failure it accepted 448 LiDAR, 516 IMU, 516
GNSS, 91 optical-flow, and 9 paper visual factors.  The source-aware map
contained 83,973 voxels: 59,303 LiDAR, 34,419 RGB-D, 9,749 joint-source, and
49,554 LiDAR-only voxels.  RGB coverage was 0.164393; RGB-D contributed 24,670
supplementary voxels (0.415999 volume growth).  Recorded geometry conflicts,
conflict ratio, ghosting proxy, and evictions were all zero.

This full-stack test is **FAIL**, not a stability pass.  During the second turn
ArduPilot reported `Crash: AngErr=50>30, Accel=0.2<3.0`; the vehicle was safely
disarmed without LAND.  The backend recorded 27 non-committed transactions and
27 rollbacks (22 excessive translation corrections and 5 excessive
accelerometer-bias corrections).  Simulation RTF was 0.478.  The failure
occurred after only about 5.8 m of the first leg, so long-duration map stability
was not demonstrated even though the partial map was internally consistent.
All spawned processes and ports were cleaned after the run.

## Release decision

Robustness V3 is suitable as a reproducible fault-injection and regression
candidate, but it has **not** reached the gate for direct real-hardware flight
integration.  Blocking evidence is the nonzero rollback/full-stack crash, the
absence of a demonstrated nonzero Native LiDAR time-offset margin, and the
missing live GNSS innovation/recovery validation.  The next safe step is a
tethered or propeller-off hardware bench run that verifies clock discipline,
MID360 extrinsics, FCU GNSS innovation metadata, and transaction integrity
before any flight test.

Machine-readable evidence is generated under ignored `logs/tmp` directories;
the campaign aggregate is `logs/tmp/robustness_v3_campaign_summary.json` and
the final map result is
`logs/tmp/robustness_v3_joint_map_stress_final6/robustness_joint_map_report.json`.
