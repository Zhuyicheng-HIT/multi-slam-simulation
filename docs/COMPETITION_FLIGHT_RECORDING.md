# Competition flight recording

These tools add a repeatable recording contract around the existing onboard
runtime. They do not change estimator mathematics, raw Livox topics, Safety,
ExternalNav, or command ownership.

## Rules

- Select the mode and restart the runtime **before takeoff**. Do not switch
  estimator sources in flight.
- Raw `/livox/lidar` and `/livox/imu` are always recorded and never modified.
- Stop only after landing/disarm. Press `Ctrl+C` once and wait for metadata,
  validation JSON, and SHA256 generation.
- Metric 3 may be triggered in flight, but only through the owned manual
  service. It never publishes `/relocalization/request` or a MAVROS setpoint
  directly.

The public commands are:

```bash
bash tools/competition_metric1.sh
bash tools/competition_metric2_full.sh
bash tools/competition_metric2_no_camera.sh
bash tools/competition_metric3_four_source.sh
bash tools/competition_metric3_five_source.sh
```

`competition_metric3.sh` remains an alias of the five-source command.

By default they attach to an already-running, correctly configured onboard
runtime and refuse to record when required topics are absent. To let the tool
start the validated runtime itself, set `COMPETITION_RUNTIME_COMMAND` to that
machine's unchanged launcher. The mode is exported before the launcher runs.
For example:

```bash
export COMPETITION_RUNTIME_COMMAND='bash /absolute/path/to/validated_launcher.sh'
bash tools/competition_metric1.sh metric1_take01
```

Do not point this at an unverified launcher. The repository intentionally does
not guess the location of the separate hardware-driver project.

## Modes

| Command | Estimator sources | Other behavior |
|---|---|---|
| `competition_metric1.sh` | medium-degraded Native LiDAR + MID360 IMU + optical flow | GNSS and visual standardized outputs must be absent |
| `competition_metric2_full.sh` | nominal LiDAR, IMU, GNSS, flow, RGB-D | dynamic observer side channel on |
| `competition_metric2_no_camera.sh` | nominal LiDAR, IMU, GNSS, flow | visual off; dynamic observer side channel on |
| `competition_metric3_four_source.sh` | nominal LiDAR, IMU, GNSS, flow | visual off; persistent database and readiness required |
| `competition_metric3_five_source.sh` | all five nominal sources | visual on; persistent database and readiness required |

For Metric 1 the software profile must report `lidar_medium`. When a genuinely
degraded physical LiDAR setup is used instead, explicitly set
`COMPETITION_PHYSICAL_LIDAR_DEGRADATION=1`; this flag is recorded in the run
configuration and must not be used to bypass a software test accidentally.

## Metric 2 event markers

Mark the evaluator-only ground-truth moments from a second terminal. These
messages do not enter the detector or estimator:

```bash
python3 tools/competition_event.py --mode metric2-full --event DYNAMIC_ENTER --detail person_1
python3 tools/competition_event.py --mode metric2-full --event DYNAMIC_EXIT --detail person_1
```

Run both full and no-camera trials. This directly measures whether RGB-D helps
under the actual onboard CPU budget rather than assuming it does.

## Metric 3 manual control

Check readiness before flight:

```bash
bash tools/manual_relocalization.sh status
```

During the selected test segment:

```bash
bash tools/manual_relocalization.sh start
```

Emergency/operator cancellation:

```bash
bash tools/manual_relocalization.sh cancel
```

`START` is refused unless the persistent keyframe database is ready. Success is
not one candidate match: the bag must contain the accepted result, matching
`FusionEpoch`, recovery-validation state, and resumed mission state.

Without propellers or flight, the control ownership path can be checked in an
isolated ROS domain:

```bash
bash tools/manual_relocalization_no_flight_smoke.sh
```

That smoke test does not claim a localization success rate. The competition
success rate still requires repeated moving trials.

## Outputs

Each run is written under `~/multi-slam-competition-runs/<run-id>/` by default:

- `mode.txt` and `mode.env`: requested source contract;
- `frequencies/summary.tsv`: simultaneous sensor-rate sample;
- `bag/`: zstd-compressed rosbag2 data;
- `bag_info.txt` and `bag_validation.json`;
- `SHA256SUMS.txt`.

The verifier fails if a required stream is empty or if a source that should be
off appears on its standardized estimator topic. Raw hardware topics may still
exist for audit, but they are not treated as accepted estimator factors.
