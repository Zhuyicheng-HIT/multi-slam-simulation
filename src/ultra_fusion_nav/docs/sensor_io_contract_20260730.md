# Four-source sensor I/O contract

Date: 2026-07-30

The estimator consumes the same normalized ROS 2 topics in simulation and on
hardware. Transport-specific adapters end before these topics. Gazebo truth,
MAVROS local position, and FCU fused GNSS/flow position remain evaluator or
control data and never feed the estimator.

| Modality | Simulation source | Hardware source | Normalized estimator input |
| --- | --- | --- | --- |
| LiDAR | Gazebo MID360 bridge | Livox SDK2 CustomMsg | `/sensors/lidar/*` and FAST-LIO native factors |
| IMU | Gazebo MID360 IMU via `/livox/imu` | MID360 IMU via Livox driver | `/sensors/imu` (`sensor_msgs/Imu`) |
| GNSS | MAVROS raw receiver observation | C2 TESTRN NMEA0183 over RS232 | `/sensors/gnss/fix`, `/sensors/gnss/raw` |
| Optical flow | MTF01P protocol-equivalent simulator | MTF-01 direct serial adapter | `/sensors/optical_flow/rad` |
| RGB-D | Gazebo D435i bridge | RealSense ROS 2 driver | reserved `/sensors/rgbd/*`, disabled by default |

## Direct GNSS

The C2 adapter is `uf_sensor_pipeline/nmea_gnss`. It expects 115200 baud, 8N1
and consumes strict NMEA0183 `$...*hh\r\n` sentences. `GNRMC` supplies fix
validity, WGS84 latitude/longitude, speed, course, UTC time, and UTC date.
`GNGGA` supplies fix quality, satellite count, HDOP, MSL altitude, and geoid
separation. The node publishes:

- `/gnss/direct/fix`: pre-fault `sensor_msgs/NavSatFix`;
- `/sensors/gnss/raw`: normalized `mavros_msgs/GPSRAW` integrity metadata;
- `/gnss/direct/time_reference`: source UTC without replacing the monotonic ROS
  arrival timestamp;
- `/gnss/direct/diagnostics`: checksum, parsing, fix, and staleness counters.

The supplied RMC example calculates to checksum `4E`, not the documented `50`.
Strict checking remains enabled for flight. `nmea_strict_checksum:=false` is
only for bench diagnosis and still increments the checksum-error counter.

Hardware launch overrides for the common sensor pipeline are:

```bash
ros2 launch uf_sensor_pipeline sensor_pipeline.launch.py \
  enable_nmea_gnss:=true \
  nmea_port:=/dev/ttyUSB0 \
  gnss_input_topic:=/gnss/direct/fix \
  enable_vision:=false
```

## D435i reservation

Vision is disabled by default. Enabling it activates the color/depth fault
adapters, contract checks, and a configurable `base_link -> d435i_link` mount
transform. The default simulated mount is `xyz=[0.20, 0.0, 0.02] m` and
`rpy=[0, 0, 0] rad`. Real mounting values must be measured or calibrated and
passed through `d435_x/y/z` and `d435_roll/pitch/yaw`; the RealSense driver owns
the camera optical-frame transforms below `d435i_link`.
