# MTF01P FCU Routing Experiment and Rollback

Date: 2026-07-29

## Decision

The default algorithm input is rolled back to the direct companion-computer path:

```text
MTF01P-like observation -> /sim/optical_flow/rad -> sensor pipeline -> fusion
```

The ArduPilot routing implementation remains available only as an opt-in experiment:

```bash
FLOW_TRANSPORT=fcu_router bash src/ultra_fusion_nav/scripts/run_lio_baseline_experiment.sh
```

The reason is not MAVLink packet loss. The route passed packet-level validation, but
long WSL simulation runs exposed MAVROS telemetry startup and host-clock correction
instability. That instability can prevent time association with FCU IMU or invalidate
the independent FAST-LIO reference. It is not a sound basis for adding more
simulation-specific estimator logic.

## Protocol Result

For the MTF01P `mavlink_apm` input path used by ArduPilot, the sensor-side messages are:

- MAVLink 1 `OPTICAL_FLOW`, message id 100.
- MAVLink 1 `DISTANCE_SENSOR`, message id 132.
- Sensor system id 200 was used so packets can be distinguished from FCU-generated
  telemetry with system id 1.

`OPTICAL_FLOW_RAD` visible in a ground station is FCU telemetry and must not be
mistaken for the raw MTF01P input message.

References:

- MicoAir MTF01P manual: <https://micoair.cn/zh/docs/sensors/sensors/mtf-01p-sensors>
- ArduPilot MTF01 setup: <https://ardupilot.org/copter/docs/common-mtf-01.html>
- ArduPilot MAVLink routing: <https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html>

## ArduPilot 4.8-dev Mapping Used in SITL

```text
SERIAL0 -> MAVROS companion link, tcp:5760
SERIAL1 -> MTF01P sensor link, tcp:5762
SERIAL2 -> independent flight-command test link, tcp:5763

SERIAL0_PROTOCOL = 2
SERIAL1_PROTOCOL = 1
SERIAL1_BAUD     = 115
FLOW_TYPE        = 5
RNGFND1_TYPE     = 10
MAV1_OPTIONS     = 0   # SERIAL0 backend
MAV2_OPTIONS     = 0   # SERIAL1 backend
```

In this ArduPilot checkout, `MAVn_OPTIONS` bit 1 is `NO_FORWARD`, numeric value 2.
It must remain clear for routing. Older vendor instructions using
`SERIALn_OPTIONS=1024` intentionally disable forwarding and therefore conflict with
the companion-routing objective.

## Measured Route

Packet-level standard-route result from
`/tmp/uf_flow_route_standard_20260729_v2`:

| Metric | Result |
|---|---:|
| Routed flow rate | 21.93 Hz |
| MAVLink source period | 33 ms |
| Source timestamp present | 100% |
| Source timestamp regressions | 0 |
| Decode/framing errors | 0 |
| Round-trip rate ratio | 0.975 |
| Quantization-aware correlation | 0.9955 |
| Flow-rate scale | 0.9858 |
| FCU gyro compensation coverage | 99.73% |
| Routed ROS stamp regressions | 0 |

The simulator produces about 18-23 Hz because Gazebo does not sustain the sensor's
nominal hardware rate. No synthetic upsampling was added. Real MTF01P rate and clock
behavior still require a serial/MAVLink capture on the physical unit.

## Flight Evidence

The final short four-source probe at
`/tmp/uf_flow_route_clock_probe_20260729_v2` completed takeoff, three 90-degree turns,
rectangle translation and landing with exit code 0:

- Unified-backend ATE RMSE: 0.0503 m.
- RPE translation RMSE: 0.0115 m.
- RPE rotation RMSE: 0.574 deg.
- Flow factors added: 34.
- Flow observations disabled during turning/recovery: 142.
- Route validation: passed.

Long standard runs were not stable enough to retain as a baseline:

- One run had no usable flow factors because of a persistent clock-domain offset.
- A later run added 149 flow factors, but the independent FAST-LIO yaw RMSE reached
  18.2 degrees and the truth/FCU-gyro reference correlation collapsed.
- After a clean WSL restart, one startup produced no `/mavros/imu/data_raw` because
  all telemetry stream requests timed out.

These failures remain recorded. They are the reason the FCU route is opt-in rather
than the default path.

## Hardware Follow-up Gate

Do not enable FCU routing on the aircraft until all of the following are captured:

1. Actual MTF01P system/component id and wire message ids.
2. Actual output rate and `time_usec` semantics.
3. ArduPilot firmware-specific `MAVn_OPTIONS` to physical `SERIALn` mapping.
4. Zero packet duplication and monotonic companion timestamps over at least 10 min.
5. FCU IMU association coverage above 99% without host-arrival timestamp substitution.
