# MID360S real/simulation protocol alignment (2026-07-29, corrected 2026-08-29)

## Scope

This audit compares the connected MID360S SDK2 UDP stream, the real ROS 2
driver output, and the Gazebo-to-Livox simulation bridge. Ground truth is not
used by the localization algorithm.

## Observed Windows stream

The Windows probe received the following stream from `192.168.1.123` on host
`192.168.1.50`:

| Stream | Source -> destination | Packet | Observed rate |
| --- | --- | --- | --- |
| Point cloud | `56300 -> 56301` | 1380 bytes, 96 points, data type 1 | 2010 packets/s, 2.645 MiB/s |
| LiDAR IMU | `56400 -> 56401` | 60 bytes, 1 sample, data type 0 | 198.8 Hz |

Point data type 1 contains Cartesian `int32` millimetre coordinates,
reflectivity and tag. The SDK2 ROS driver converts coordinates to metres and
publishes `livox_ros_driver2/msg/CustomMsg`.

The observed `time_type` was 0. Timestamps therefore use LiDAR boot time and
are not yet synchronized to the FCU or host clock.

## ROS interface contract

| Role | Topic | Type | Frame |
| --- | --- | --- | --- |
| MID360S points | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | `livox_frame` |
| MID360S internal IMU (raw) | `/livox/imu` | `sensor_msgs/msg/Imu` | `livox_frame` |
| MID360S internal IMU (backend copy) | `/sensors/imu` | `sensor_msgs/msg/Imu` | `base_link` |
| FCU IMU | MAVROS/ArduPilot chain | separate | FCU-defined |

The previous `/livox/lidar_imu`/FCU ownership statement was incorrect. The
MID360S internal `/livox/imu` is retained at approximately 200 Hz and remains
the FAST-LIO inertial input. The Ultra-Fusion backend consumes its SI/body-FLU
copy on `/sensors/imu`; the FCU IMU is a separate chain and is not substituted
for it. FAST-LIO's factory LiDAR-to-IMU extrinsic remains independent from the
whole-module 15-degree body installation rotation.

The project launch now generates its own MID360S runtime SDK2 JSON with the
actual LiDAR and host addresses. It no longer includes the generic MID360
launch file whose installed template used `192.168.1.12` and host
`192.168.1.5`.

## Simulation corrections

The simulation bridge now matches the SDK2 ROS contract in the fields consumed
by FAST-LIO:

- four logical lines, matching SDK2 `kLineNumberMid360 = 4`;
- `tag = 0` for nominal high-confidence points;
- monotonically increasing `offset_time` over a 100 ms, 10 Hz scan;
- at most 20,000 points per scan, close to the observed 193,000 points/s;
- metre coordinates in `CustomPoint`;
- `mid360_link` frame and `/livox/lidar` CustomMsg output.

## Intentional remaining differences

1. Gazebo emits an organized angular grid. A real MID360S uses a
   non-repetitive Livox sampling pattern. Logical line and timing fields can be
   matched, but the exact ray sequence cannot be reproduced by the current
   Gazebo sensor.
2. Real reflectivity and tag values depend on material, rain, fog, dust and
   multi-path effects. Nominal simulation uses tag 0 and requires explicit
   fault injection for those effects.
3. Real SDK2 UDP packet loss, reordering and 96-point packet boundaries are not
   visible after conversion to ROS CustomMsg. Separate network fault tests are
   required if those failure modes matter.
4. Simulation currently stamps the scan in the host/ROS time domain. The real
   unit is still on unsynchronized boot time (`time_type = 0`). PTP or PPS/GPS
   synchronization must be validated before FCU-IMU tight coupling.

## Verification status

- Protocol helper tests: passed, 3/3.
- Synthetic PointCloud2 -> Livox CustomMsg contract test: passed.
- `multi_slam_uav_sim` and `mid360_reliable_mapper` build: passed.
- ROS package regression: 13 reported test results, no failures; existing
  package suite printed 35 successful cases.
- Live SDK2 config parsing: passed and detected LiDAR type 8.
- Live WSL ROS point reception: pending. During the final launch attempt the
  Windows Ethernet adapter was `Disconnected`, WSL `eth0` was `DOWN`, and the
  LiDAR no longer answered ping, so SDK socket binding could not be validated.
