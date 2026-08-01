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

The read-only RTAB database diagnostics for the rectangle run found 106 nodes,
33 keyframes, 17,583 feature rows and five map IDs. No global or proximity loop
closure was accepted in that short front-facing single-lap run. The loop and
cross-session mechanisms are present and tested, but accepted relocalization is
not claimed by this runtime result.

## Remaining external/runtime gaps

- `/home/zyc/multi-slam-deps/mid360_ws` does not contain PR #6's
  `NativeLidarFactor` patch. Formal defaults remain native-factor mode. The local
  validation runner detects the missing interface, selects `lio_pair`/pose
  fallback, and disables the backend IMU factor so FAST-LIO's internal IMU update
  is not counted twice.
- Native-factor mode must be rerun after building the supplied FAST-LIO patch in
  the dependency workspace.
- A controlled two-session route is still required to demonstrate an accepted
  cross-session loop/relocalization with this integrated branch.
- WSLg hardware rendering is unavailable in this environment; Gazebo used
  `kms_swrast` and occasionally required a clock-bridge restart. The runner now
  performs one owned retry without restarting Gazebo or FCU.

## Draft PR decision

The branch is suitable for a new **Draft PR**: the build/tests pass and the
visual factor is demonstrably in the unified backend. It should remain draft
until patched native-factor FAST-LIO and a successful two-session relocalization
run are attached. It is not ready to merge or mark ready for review yet.
