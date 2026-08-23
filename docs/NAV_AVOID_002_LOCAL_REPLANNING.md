# NAV-AVOID-002: Local Avoidance and Safe Replanning

## Scope and baseline

This stage starts at `feat/raw-obstacle-safety-slice-v1` commit
`06894236874ccd173019ea1c541a578e2d928e11`. It does not change estimator
mathematics, Z-COV, Dynamic Clean ownership, ExternalNav, or relocalization.
The existing raw obstacle monitor and sole flight-command arbiter remain the
authority boundaries.

## IMPACT / EGO audit

The audited IMPACT `main` was `db3686fd23c017e45b4a7f3aa32535ca988982f7`.
Its `xq_ego_planner` is a real ROS 2 snapshot of ZJU EGO-Planner (upstream
`ego-planner-swarm`, `ros2_version`, commit `23a8d5a...`, GPL-3.0), including
the C++ grid map, A*, B-spline optimizer, trajectory server, and planner FSM.
IMPACT adds ROS-clock handling, parameterized target input, requested-Z
preservation, and Z inflation. Its continuous collision callback,
emergency-stop/replan semantics, obstacle inflation, and corner replanning are
production-relevant concepts.

IMPACT's `SentinelFSM` is a deterministic Python policy core, while
`p5_ego_command_node.py` is a SIL adapter that directly publishes MAVROS. The
published P5 evidence is SIL/proxy validation; P12/P13 dynamic and latency work
is not a completed production flight stack. The SIL adapter and whole GPL stack
were therefore not copied. This stage adopts the verified semantics through a
small project-native planner that preserves the existing command ownership.

## Architecture and ownership

```text
Raw MID360 CustomMsg ----------------+------------------------------+
                                     |                              |
                                     v                              v
                          obstacle safety monitor       local avoidance planner
                                     |                   (bounded 2.5-D A*)
                                     |                              |
                                     |                     candidate path +
                                     |                     planner pose intent
                                     +------------+-----------------+
                                                  v
                                      flight_command_arbiter
                                      (only MAVROS publisher)
                                                  |
                                                  v
                                  /mavros/setpoint_position/local
```

Dynamic Clean remains localization-only. The planner and safety monitor both
consume the configured Raw MID360 topic. The planner never publishes MAVROS;
the raw safety state can veto planner and active-relocalization intents at any
cycle. Priority remains manual/FCU failsafe, obstacle BRAKE/HOVER, LAND/RETURN,
localization HOLD, safe active relocalization, local planner, then mission.

## Planner and safety contract

The planner creates a local XY grid around the current pose and clipped mission
goal, downsamples Raw points, applies the configured LiDAR-to-body extrinsic,
and ignores points outside the vehicle vertical collision band. An 8-connected
A* forbids diagonal corner cutting. Line-of-sight simplification and waypoint
spacing preserve the obstacle inflation. Planning is bounded by a 7 m horizon,
30,000 expansions, and 80 ms wall time.

The state sequence is:

```text
NAVIGATING -> PATH_BLOCKED -> BRAKE_HOLD -> REPLAN
           -> TRAJECTORY_VERIFY -> RESUME
```

Raw freshness, finite/timestamp checks, pose/mission freshness, search success,
and a second Raw trajectory verification are mandatory. Failure at any gate
holds the current pose in `HOVER_REQUIRED`; no empty command or unverified
trajectory is forwarded. Planning inflation is 0.80 m and independent
verification clearance is 0.65 m. The additional 0.15 m is tracking/discrete
grid margin, not a relaxed safety threshold.

## Verification

The deterministic matrix ran 20 trials per scenario:

| Scenario | Result | Collisions | Replan P95 (ms) |
|---|---:|---:|---:|
| Frontal wall | 20/20 | 0 | 1.878 |
| Single column | 20/20 | 0 | 1.819 |
| L-shaped corner | 20/20 | 0 | 2.186 |
| Narrow passage | 20/20 | 0 | 4.106 |
| Sudden obstacle | 20/20 | 0 | 1.403 |
| New path blockage | 20/20 | 0 | 1.381 |
| Continuous replanning | 20/20 | 0 | 1.479 |
| Planner failure (expected hover) | 20/20 | 0 | fail-closed |
| Localization loss plus obstacle | 20/20 | 0 | fail-closed |
| Relocalization/obstacle conflict | 20/20 | 0 | obstacle owns |

The 400-plan stress benchmark produced 400 verified paths, zero collision,
1.185/1.697/1.834 ms P50/P95/P99, and 0.805 m minimum realized clearance.
The ROS closed-loop wall test passed 3/3, reaching the original goal in
3.76--3.82 s after one replan per run with the complete six-state transition
and zero collision. Topic graph
inspection found exactly one setpoint publisher: `flight_command_arbiter`.
The inherited Safety Slice smoke also passed CLEAR, BRAKE, dropout HOVER,
localization HOLD, non-finite planner rejection, relocalization veto, and sole
publisher checks.

The planner-only stress process used 4.1 MiB maximum RSS. During the ROS
closed loop, the planner sampled at about 1.0% CPU and 25.6 MiB RSS; the raw
monitor and arbiter sampled at about 1.3%/25.5 MiB and 1.0%/26.6 MiB. The full
workspace built 19 packages and passed all 178 tests. Python, YAML, XML, shell,
and `git diff --check` validation also passed.

## Failure behavior and limits

- Raw scan stale/dropout, timestamp regression, sparse/non-finite scan: hover.
- Pose or mission stale/non-finite, or localization hold: hover.
- Planner timeout/no path/start or goal in collision: hover and retry only when
  healthy inputs continue; never force the mission command through.
- Candidate becomes blocked before or during execution: brake, hold, and replan.
- Manual/FCU failsafe releases automatic ownership to the flight controller.

This is local goal-to-goal avoidance, not frontier exploration. The obstacle
representation is scan-local and conservative; complex real MID360 occlusion,
vehicle dynamics, aerodynamic braking, moving-obstacle prediction, and hardware
latency still require propeller-off bench and tethered-flight validation before
production flight.
