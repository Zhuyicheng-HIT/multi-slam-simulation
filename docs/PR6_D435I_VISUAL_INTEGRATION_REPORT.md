# PR #6 D435i visual SLAM integration report

## Immutable inputs

- PR #6 branch: `feature/ultra-fusion-stage3`
- PR #6 HEAD: `8bdad3daf53a905bea344d6720c72b3edb9277d1`
- Visual source ref: `f14f05c310b565acec5fd8f20386c6f0eeeabf4a`
- Integration branch: `feat/pr6-d435i-visual-integration`
- Fresh worktree: `/home/zyc/projects/multi-slam-simulation-pr6-d435i-integration-20260801`

Neither input repository was modified. No branch was pushed and no pull request was
created or changed.

## Selected architecture

The PR #6 sensor pipeline remains the owner of normalized `/sensors/*` topics,
fault injection, the `base_link -> d435i_link` mount, reliability scheduling,
FAST-LIO, GNSS, optical flow and the unified backend. The migrated C++ bridge owns
the raw `/front/d435i/*` RGB-D contract and only the camera optical-frame TFs.

RTAB-Map consumes normalized color and depth plus the raw `CameraInfo`. D_V_rgbd is
the sole publisher of `/reliability/vision_score`. The unified backend samples
`/rtabmap/odom` by arrival time and converts it to an origin-free relative SE(3)
factor. Image features are reliability evidence only; they are not inserted as a
second visual factor. This prevents the same visual evidence from being counted
both as features and as RTAB odometry.

## Migrated visual functionality

- C++ Gazebo-to-ROS D435i bridge with exact RGB/depth pairing, `16UC1` depth,
  `CameraInfo`, camera/IMU optical TFs and bounded best-effort QoS.
- RTAB-Map RGB-D odometry and SLAM launch/configuration, persistent databases,
  loop-closure configuration and localization/cross-session profiles.
- D_V_rgbd reliability state, metrics, diagnostics and PR #6
  `uf_interfaces/ReliabilityScore` output.
- D_V-weighted RTAB relative factor support in both linear and manifold backend
  windows, including hard motion/freshness gates and diagnostics.
- Cross-session monitoring, comparison, database diagnostics and relocalization
  motion helpers.
- Owned-process lifecycle scripts, launch smoke checks, unit tests and operator
  documentation.

## Deliberately omitted or disabled

- The D435i point-cloud demonstration was not migrated. Point-cloud generation in
  the bridge remains disabled in the integrated launch and had no runtime topic.
- Ultra-Fusion official-binary adaptation experiments were not migrated.
- Failed combined-bag and MID360 timestamp experiments were not migrated.
- Source-tree backends, schedulers, sensor bridges and reliability nodes already
  implemented by PR #6 were not copied.
- PR #6's lightweight internal image score is disabled for this launch; after the
  fix it does not even create a second vision-score publisher.

## Conflicts resolved

- **TF ownership:** PR #6 publishes `base_link -> d435i_link`; the C++ bridge was
  limited to child optical frames, eliminating the duplicate mount TF.
- **Topic ownership:** the C++ bridge publishes raw camera topics while PR #6
  alone normalizes them to `/sensors/rgbd/*`.
- **Visual weighting:** one D_V score gates one RTAB relative factor. No raw
  feature factor or second visual score is used.
- **Clock domains:** RTAB uses simulation time, while the backend samples RTAB
  odometry on steady-clock arrival because PR #6 LIO/GNSS use wall-clock stamps.
- **Optional transport tracking:** D_V now uses the actual normalized exact image
  pairs as its observation count; optional transport tracking is used only for
  source-drop diagnostics.
- **Lifecycle:** cleanup validates and signals complete owned ROS launch process
  groups, including the explicit dependency fallback launches.

## Build and automated tests

- Full sequential `colcon build --symlink-install`: 12 packages built; the D435i
  C++ bridge compiled and linked against Gazebo Transport/messages.
- Full `colcon test --return-code-on-test-failure`: 17 test jobs, zero errors,
  zero failures and zero skipped.
- Backend Python suite: 73 tests passed, including visual increment, window and
  manifold factor cases.
- Final affected-package regression: 47 `multi_slam_uav_sim` tests and 42
  `uf_reliability` tests passed.
- Migrated visual helper tests: 12 passed. Lifecycle short test passed.
- Both new launches expand with `--show-args`; Python compile, shell syntax and
  `git diff --check` pass.

## Headless runtime evidence

The principal flight run is
`logs/pr6_d435i_visual/runtime_20260801_full5`. It ran in an isolated ROS domain
with CPU/software rendering explicitly allowed because WSLg could not open
`/dev/dri/renderD128`.

- Raw color, aligned depth and `CameraInfo` each had one publisher.
- Normalized color and depth each had one publisher.
- Depth was `16UC1`, 640x480, in `front_d435i_color_optical_frame`.
- RTAB RGB-D odometry used exact synchronization and reported normal feature
  quality (typically about 70-86 tracked features in the inspected interval).
- The D435 point-cloud demo topic did not exist.
- FAST-LIO `/Odometry`, Livox input, MAVROS IMU/GNSS and optical-flow topics were
  live.
- Unified-backend diagnostics reached 453 LiDAR, 447 GNSS and 8 optical-flow
  factors with zero optimization errors during the flight evidence capture.
- The 6x4 m `small_rectangle` completed all four sides and four 90-degree turns;
  the landing command was accepted.

After correcting D_V's observation-count source, the focused run
`logs/pr6_d435i_visual/runtime_20260801_visual_factor2` produced:

- `published=334`, `lio_pose_fallbacks=334`, `optimization_errors=0`
- `visual_received=384`, `visual_factor_attempts=32`
- `visual_factors=32`, `visual_disabled=0`
- D_V calibration with nonzero exact-pair rate (`frame_hz=4.4865`)
- exactly one `/reliability/vision_score` publisher:
  `d435i_visual_reliability`

Thus visual odometry is genuinely consumed by the PR #6 unified backend; it is
not merely launched beside it.

The earlier rectangle database contained 106 nodes and five map IDs because
the high-altitude, turning flight repeatedly lost visual odometry. RTAB's logs
show automatic odometry resets followed by `Increment map id`; the database
contained only neighbor links. This was a route/observation failure, not a
cross-session database result.

## Native LiDAR factor result

PR #6 itself contains the reproducible FAST-LIO patch, message contract,
launcher switch and validator. The export is disabled by default. The local
dependency checkout is the required upstream commit
`a4743b095409588842a5b30ddfa27e29d2f99164`, but its existing install is
unpatched. To keep that source read-only, the patch was applied and built in an
ignored isolated dependency workspace under `logs/native_factor_dependency_ws_20260801`.

All five final-matrix processes reported `input_trigger=native_factor`,
zero LIO pose fallbacks and zero optimization errors. The reference session
relinearized 723 native factors and inserted 94 visual factors. The four
localization sessions relinearized 600, 573, 577 and 585 native factors while
inserting 120, 137, 95 and 128 visual factors respectively. `/Odometry` remains
the implemented PR #6 compatibility path for an unpatched overlay, but it was
not the path used by the final matrix.

## Cross-session relocalization result

The final independent-process matrix is
`logs/d435i_visual_slam/cross_session/matrix_pr6_d435i_final_20260801_r4`.
Session 1 produced one map ID, 71 nodes, 2,534 words, 11,509 features, 18
GlobalClosure links and nine live geometry-validated closure events. Maximum
geometry support was 67 inliers; lost/reset were zero. The immutable mother
database SHA-256 was
`80f79acfc17e8c11c6d294c5f9c40db2bbe2ada1a85c9cc43e4321c502662c2b`.

Three of four independent Session 2 conditions relocalized without false
matches: same pose matched node 60 with 72 inliers, the 0.25/0.20 m offset
matched node 59 with 40 inliers, and the 15 degree yaw offset matched node 60
with 39 inliers. All matched map ID 0, had lost/reset/TF-backward-jump counts
of zero and reached stable alignment in 3.572, 2.888 and 3.045 seconds.

| Condition | Node/map | Inliers/matches | Closure translation (m) | Closure yaw | `map -> odom` correction |
|---|---|---:|---|---:|---|
| same pose | 60 / 0 | 72 / 126 | (0.0246, -0.0303, 0.0058) | 0.70 deg | 0.5036 m, 0.77 deg |
| 0.25/0.20 m offset | 59 / 0 | 40 / 66 | (0.1141, 0.2175, 0.0108) | 0.62 deg | 0.6092 m, 0.71 deg |
| 15 deg yaw offset | 60 / 0 | 39 / 73 | (0.0082, 0.0271, 0.0076) | 15.64 deg | 0.5041 m, 15.82 deg |
| 180 deg reverse | none | 0 / 0 | none | none | none |

The 180 degree reverse view produced 17 candidates, all rejected, and no
accepted event. It is an honest algorithm failure, not a false relocalization.
The mother hash was unchanged after every child session, all active markers and
PID manifests cleared, and the audited FCU/Gazebo ports were free.

The per-session working database SHA-256 values were
`4eb60239e529316579515edaac9f97f043d62f93238578523001d7bab6c4fdd0`
(same pose),
`613e9a004b2980b930b34f39fe747008357e88065afdc76204f0bbd1d0c6306f`
(position offset),
`ca535f06514efa84ae1de35207145f50373c5bca41297ea3248119d39d8b1aa2`
(yaw offset), and
`4f8ca7a6121e152ae9b3b7db8f0e0588ebcb5ffec87d2438432957f76a1d71e2`
(reverse view).

## Remaining runtime gaps

- A normal machine setup must apply the checked-in native-factor patch before
  building the external FAST-LIO workspace; the existing dependency checkout
  was intentionally not modified by this validation.
- Direct 180 degree appearance reversal is not supported by this front-facing
  single-camera reference map. It failed safely with no accepted wrong match.
- WSLg rendering required temporary access to `/dev/dri/renderD128`; its
  original `render` group ownership was restored after the matrix.

## Draft PR decision

The branch is suitable to push and open as a new **Draft PR**. Native-factor
FAST-LIO, D_V-weighted visual factors and three real cross-session localization
conditions are now demonstrated in the PR #6 stack. It should remain draft
while the 180 degree reverse-view limitation and external dependency patching
procedure are reviewed; it is not being marked ready or merged here.
