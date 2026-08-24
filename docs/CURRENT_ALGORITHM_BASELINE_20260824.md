# Current algorithm baseline (2026-08-24)

## Read this first

There are two intentionally separate lines of work. Do not treat the LiDAR
directional experiment as the production baseline.

| Purpose | Branch | Exact commit | Tag / decision |
| --- | --- | --- | --- |
| Upstream PR #14 | `feat/core-algorithm-cleanup-20260817` | `5f15ab032949e24b539375b4bfa6349e6b562b3b` | GitHub PR #14, stacked on `feat/five-source-stage3-integration` |
| Stable complete system | `integration/current-complete-pr14-20260824` | `054a6744cf2265bd4dc1bd4cee0be6287cd2dbc1` | `current-complete-pr14-20260824` |
| Experimental LiDAR directionality | `feat/lidar-directional-reliability-v1` | `90e8ff3e429cbf873c94b51ca65cb03d2aacdb0e` | `lidar-dir-001-directional-reliability-20260824`; **DO_NOT_PROMOTE** |

The stable branch was rebuilt from the exact PR #14 head. It selectively
replayed the locally validated Dynamic, safety, avoidance and relocalization
work; it did not merge or rewrite PR #15. PR #14 and PR #15 remain open Draft
PRs owned by their existing branches.

## What changed in the latest PR #14

Relative to the previous upstream commit
`e934132ffdd991b0dd59a752eead93d2e0313b40`, PR #14 added one commit and changed
8 files (+499/-136). It restored the tunnel/straight route contract, replaced
the simulated MicoLink optical-flow bridge with a ROS 2 C++ package, simplified
sensor startup and MAVROS stream requests, and expanded the simulated MID360
body exclusion envelope to X/Y `[-0.45, 0.45] m` and Z `[-0.35, 0.15] m`.

That geometry change is important: directional LiDAR results collected with
the older envelope are not valid evidence for the current PR #14 baseline.

## Stable complete-system capability state

### Five-source localization and reliability

- NativeLidarFactor, IMU, GNSS/BDS, optical flow and D435i visual factors enter
  the unified backend under one-observation-one-factor ownership.
- ExternalNav ownership and the estimator integrity/transaction gates are
  unchanged.
- The production LiDAR reliability path remains the scalar scheduler. The
  directional XYZ and eigensubspace handoff switches remain disabled.
- D435i visual reliability (`D_V`/FRS), paper-style visual integration and the
  existing sensor health/rejection paths are present.

### Dynamic stack

- Class-agnostic observer v2 separates static, dynamic and unknown evidence
  using causal MID360 ray/free/occupied history.
- The Clean Gateway preserves original Livox point timestamps and metadata,
  keeps UNKNOWN by default, and passes the exact Raw scan on stale state,
  missing causal IMU coverage, timestamp regression, overflow or internal
  failure.
- The long-term map admits only `STATIC_CONFIRMED` geometry and refines ghosts
  without treating temporary occlusion as free space.
- Raw MID360 remains the production `/livox/lidar` input. Clean is a reversible
  localization candidate and is never the obstacle-safety representation.

### Relocalization

- Passive/automatic relocalization retains epoch/reset and transaction gates.
- Hybrid relocalization uses the refined static map for candidate search and
  Clean geometry for final registration.
- FAST-LIO reset/reinit clears or rebuilds Dynamic history safely; current or
  future Raw-branch poses are not fed into the Clean branch.

### Safety, local avoidance and active relocalization

- `raw_obstacle_safety_monitor` uses Raw MID360 and produces CLEAR, CAUTION,
  BRAKE and HOVER_REQUIRED. It fails closed on stale, non-finite or invalid-time
  obstacle data.
- `flight_command_arbiter` is the sole production MAVROS automatic position
  setpoint publisher. Manual/FCU failsafe remains above project automation;
  obstacle BRAKE/HOVER vetoes active relocalization, planner and mission.
- Local avoidance implements brake/hold, bounded replanning, trajectory
  verification and resume. A planner never publishes MAVROS setpoints directly.
- Active relocalization reuses HOLD/YAW_SCAN/safe-motion policies, passes all
  actions through the arbiter, and resumes only after epoch/result/recovery
  validation. Failure and timeout remain fail-closed.

### Map products

- The source-aware RGB-D/LiDAR online map and HybridFusion offline comparison
  remain available. HybridFusion is not the online localization backbone.
- Long-term relocalization/shared-map admission can be restricted to
  `STATIC_CONFIRMED_ONLY` geometry.

## Defaults and ownership

| Capability | Default / ownership |
| --- | --- |
| Raw MID360 to production FAST-LIO | Enabled; authoritative production path |
| Unified five-source backend and ExternalNav | Enabled by the corresponding production profile |
| Dynamic observer | Available but its integration launch is default-off |
| Clean Gateway replacing FAST-LIO input | Default-off and fail-open; never silently replaces Raw |
| Long-term static-map refinement/admission | Default-off unless explicitly selected |
| Raw obstacle safety and command arbiter | Required when the safety flight slice is launched; arbiter is sole setpoint owner |
| Local avoidance | Explicit safety/navigation launch; no direct MAVROS publisher |
| Active relocalization flight actions | Explicit policy/safety launch and recovery gates |
| HybridFusion offline map fusion | Default-off |
| LiDAR XYZ directional handoff | Default-off |
| LiDAR eigensubspace handoff | Default-off |
| Optional semantic dynamic evidence | Default-off and shadow-only by default |

The repository contains several selectable profiles, so “available” must not
be confused with “automatically launched by every profile.” Any deployment
must audit its selected launch file and parameters before flight.

## Frozen validation evidence for `054a6744`

- Latest-PR14 low-indoor rectangle: 4/4 legs, LAND and disarm.
- Native LiDAR / IMU / GNSS / flow factors: 1123 / 1134 / 571 / 293.
- Optimization errors / rollbacks / Native queue overflow: 0 / 0 / 0.
- Unified odometry: 9.999 Hz; maximum source gap 0.200 s.
- Causal 3-D RMSE / P95 / max: 2.45 / 4.32 / 6.13 cm; endpoint 2.79 cm.
- Tunnel straight startup: 1/1 waypoint, LAND/disarm, Raw MID360, Native factor
  and unified odometry present, but horizontal RMSE was 1.57 m and flow admitted
  no factors. This is a startup/contract smoke, not a tunnel-positioning pass.
- Clean overlay build: 20 packages PASS.
- Aggregate colcon result: 192 xUnit tests, 0 errors/failures/skips.
- Dynamic, Clean fail-open, static-map, Safety, local avoidance and active
  relocalization ROS smokes: PASS.
- Each safety/navigation smoke observed exactly one production MAVROS setpoint
  publisher: `flight_command_arbiter`.

Runtime evidence is intentionally kept in ignored `logs/` directories and is
not part of Git history.

## LIDAR-DIR-001 result at `90e8ff3`

The experiment separates directional reliability into
`source_health * factor_consistency * geometry_observability`, using the Native
conditional translation information after conditioning on rotation. It adds a
default-off candidate config, deterministic evaluator and diagnostics for:

- A: production-style scalar geometry reliability;
- B: map-axis XYZ conditional information handoff;
- C: arbitrary eigensubspace conditional information handoff.

The frozen-normal 13-case, five-seed algebraic matrix produced aggregate 3-D
RMSE A/B/C = 0.1015 / 0.1773 / 0.1256 m and strong-subspace RMSE = 0.0181 /
0.0058 / 0.0058 m. In the 45-degree corridor, C detected weak direction
`[0.707, 0.707, 0]` and reduced B's weak-direction error by about 43 percent.
However scalar A still had the best aggregate error in this model. The matrix
is not a full trajectory and cannot supply flight ATE/RPE.

Healthy geometric degeneracy is admitted directionally. Dropout, stale data,
future/regressing timestamps, non-finite values and contract corruption are
hard-rejected. Directional diagnostic cost P50/P95/P99 was
0.397/0.605/0.629 ms; process maximum RSS was 57.7 MiB.

Decision: **DO_NOT_PROMOTE**. XYZ cannot represent a rotated weak subspace,
eigensubspace is promising but does not yet prove an end-to-end advantage, and
there is no latest-envelope identical-input full-trajectory A/B/C replay.
Both experimental switches must remain default-off.

## Results that must not be reused as current evidence

- Any LiDAR directional or body-filter result recorded before PR #14 commit
  `5f15ab0` used different geometry and must not be compared as if identical.
- The older B3 fixed replay predates the enlarged PR #14 envelope and cannot
  close LIDAR-DIR-001.
- The short 3 m tunnel run is startup evidence only; it is not a tunnel
  localization, directional reliability or optical-flow acceptance pass.
- The deterministic directional matrix is one-step algebraic evidence, not a
  trajectory ATE/RPE benchmark.
- Historical PR #15 Dynamic metrics remain provenance for that old base; use
  the current complete branch for any new production claim.
- Z-axis Z-COV candidates previously marked DO_NOT_PROMOTE remain outside this
  baseline and must not be merged implicitly.

## Required next LiDAR work

Do not tune thresholds first. Capture each scenario once, freeze the exact Raw
sensor input, then replay that identical input independently through A/B/C:

1. normal rich 3-D;
2. 45-degree rotated corridor;
3. partial FoV / sector dropout;
4. long tunnel stress.

Each online replay must use only causal sensor data. Truth is evaluator-only.
Record 3-D and XYZ ATE/RPE, weak- and strong-direction error, detected weak
direction, eigenvalues/eigenvectors, factor admission, prediction-gate
rejection, covariance, optimizer error/rollback/overflow, solver/callback
latency and CPU/RAM. Promotion requires full-trajectory evidence without
regressing safety, estimator integrity or strong-direction information.
