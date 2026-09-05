# Gazebo-ArduPilot Takeoff Synchronization Validation

Validation branch: `fix/gazebo-lifecycle-debug-v1` (takeoff fix baseline was
`c739df7715cd4242f96fe953cddc6609671d31d2`).

## Root cause

After `CommandTOL`, ArduPilot enters Guided TakeOff. Forwarding a normal
position target calls `set_pos_NED_m()`, switches Guided to Pos while
`land_complete` is still true, and keeps the vehicle in `GROUND_IDLE` with
idle throttle/PWM. The frozen fix suppresses ordinary arbiter setpoints during
takeoff and resumes mission targets after climb confirmation.

## Startup audit

- G0: `simple_uav_test.sdf` loaded; Gazebo server and clock topic alive.
- G1 initial failure: manual invocation omitted `GZ_SIM_RESOURCE_PATH`, so
  `model://iris_apm_rgbd` was unresolved. The formal `env.sh` resource path
  fixed this; the project world then loaded and clock advanced.
- G2: official `iris_runway.sdf` with EGL headless loaded and published clock.
- G3: formal headless stack reached `/clock`, Raw LiDAR/IMU,
  NativeLidarFactor, unified odom, MAVROS and Safety/arbiter readiness.

## Flight evidence

- Final Rectangle rerun: takeoff at 1.51 m, four legs, LAND and disarm;
  `/tmp/final-routes-74/routes/rectangle2.log`.
- S-curve: takeoff at 1.53 m, six checkpoints, LAND and disarm;
  `/tmp/pr6-fast42/scurve2.log`.
- Previous Rectangle campaign: 3/3, `/tmp/rect3-campaign-127/summary.txt`.
- 300 s stability: `/tmp/flight-5min-51/runtime_evidence.json`; zero
  optimization errors, rollbacks, worker overflows and LiDAR pair timeouts.
- Final Rectangle rerun: all four legs, LAND and disarm;
  `/tmp/final-freeze-routes-1788609607/rectangle.log`.
- Final S-curve rerun: seven 2 m checkpoints, LAND and disarm;
  `/tmp/final-freeze-scurve-fixed-1788611639/scurve.log`.

## External plugin patch

Recreate the unvendored patch with:

```bash
git -C /home/ld666/ardupilot_gazebo diff --binary \
  082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5 -- src/ArduPilotPlugin.cc \
  > patches/ardupilot_gazebo-082a0fe-frame-sync.patch
cmake --build /home/ld666/ardupilot_gazebo/build -j2
```

Patch SHA256: `178b4a4acb1349ad43b26d68037728731e502cf8fcc64f7e3b53d6c7f8135e4a`

Plugin SHA256:
`1d1dec76ab650437f8e6455e4cf4e8eb29746f8d036ab40d3f00f1feb1cd30b3`

## Lifecycle root cause and repair

Two independent lifecycle defects caused the apparent Gazebo startup failures:

1. Fast DDS cross-process discovery was broken in the server session. Even the
   ROS demo talker had zero visible publishers. The installed CycloneDDS RMW
   passed the same probe immediately, so validation now explicitly uses
   `rmw_cyclonedds_cpp` and an isolated `ROS_DOMAIN_ID`.
2. `gazebo_clock_bridge` registered the high-rate Gazebo callback before its
   callback state was initialized, and published into rclpy directly from the
   Gazebo transport thread. The bridge now initializes state first, caches the
   latest stamp under a lock, and publishes from a 1 ms ROS executor timer.

Ten post-fix cold starts used separate ROS domains, Gazebo partitions, log
directories and process groups. All 10 produced native Gazebo clock and ROS
`/clock`, remained alive through readiness, and exited without Gazebo, bridge,
SITL or MAVROS residue. Evidence: `/tmp/gz-lifecycle10-nSAo/summary.tsv`.

## Controlled A/B/C

All probes published only `/autonomy/intent/mission/pose`; none published a
MAVROS setpoint. Safety Slice reported one command-decision publisher, one raw
safety publisher and exactly one MAVROS setpoint publisher in every trial.

| Mode | Revision | CommandTOL | Result | Max altitude | Max PWM |
| --- | --- | --- | --- | --- | --- |
| A1-A3 | `0996239` | yes | 3/3 stayed grounded | 0.190 m each | 1100 each |
| B1-B3 | fixed | yes | 3/3 took off | 2.35, 2.32, 2.33 m | 1579 each |
| C1-C3 | fixed | no | 3/3 stayed grounded | 0.190 m each | 1100 observed in C3 |

Every trial reached `/clock`, SITL, MAVROS, Safety and Arbiter readiness and
ended disarmed. Results are in `/tmp/abc-final-{A,B,C}-{1,2,3}/result.json`.

## Final verification

- 22/22 packages built.
- Full `colcon test` passed: 266 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check` passed and simulation processes were cleaned.
- The earlier 300 s stability evidence remains valid; the clock change does
  not alter flight, fusion, Safety, Dynamic or relocalization algorithms.
