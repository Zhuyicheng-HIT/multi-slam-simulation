# Gazebo-ArduPilot Takeoff Synchronization Validation

Frozen code: `c739df7715cd4242f96fe953cddc6609671d31d2` on
`fix/gazebo-ardupilot-actuation-sync-v1`.

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

## Verification and remaining gate

- 22/22 packages built.
- 266/266 tests passed; 0 errors, 0 failures, 0 skipped.
- `git diff --check` passed and simulation processes were cleaned.
- Strict three-valid-trials-per-mode remains open: valid evidence is A=1,
  B=2, C=0. Later C attempts were `ENV_START_FAILURE` at Gazebo
  initialization and are excluded. The old probe bypassed arbiter and is
  excluded. This report freezes the takeoff fix without claiming a false
  A/B/C pass.

The official headless G2 shutdown emitted an external `ardupilot_gazebo`
segmentation fault after the server had reached ready; startup and `/clock`
remained valid. The G3 formal stack reached all required readiness gates, but
the outer campaign timeout killed FAST-LIO during cleanup. These are recorded
as environment/cleanup evidence, not as successful A/B/C flight trials.

Additional isolation showed that the apparent `dumped core` was emitted during
timeout/cleanup: an unbounded Gazebo server run remains alive and
`gz topic -e` receives a valid `/world/low_indoor_apm_rgbd_mid360/clock` sample.
The remaining hard blocker is the ROS `gazebo_clock_bridge` boundary: the
bridge announces the correct Gazebo topic but the wrapper receives no ROS
`/clock` sample before its startup gate expires. Until that transport boundary
is made reliable, B cannot provide a valid actuator trial and C remains
unvalidated; these attempts are `ENV_START_FAILURE` and excluded from counts.

An additional clean-plugin run isolated the timing: the Gazebo process exited
before the ROS bridge could produce a sample (the bridge remained alive and
only logged its subscription). A separate unbounded run did expose the Gazebo
clock over `gz transport`, so the failure is intermittent process/plugin
lifetime behavior rather than a missing world clock topic. The latest clean
run records `terminate called without an active exception` in the Gazebo log
before the bridge can forward ROS `/clock`; this is the first abnormal event
in that trial.
