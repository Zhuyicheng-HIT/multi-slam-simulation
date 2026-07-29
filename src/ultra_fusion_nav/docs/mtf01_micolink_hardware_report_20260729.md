# MTF-01 MicoLink Hardware Protocol Report

Date: 2026-07-29

## Outcome

The MTF-01 connected to Windows `COM17` was inspected read-only at
`115200 8N1`. It is currently sending MicoAir's MicoLink protocol, not MAVLink.
The default companion-computer architecture is therefore:

```text
MTF-01 MicoLink serial -> Windows COM17 -> read-only TCP bridge
  -> WSL MicoLink decoder -> /hardware/mtf01/optical_flow/rad
  -> sensor pipeline -> reliability scheduler -> fusion backend
```

The simulator now uses the same binary boundary:

```text
Gazebo image/LK output -> /sim/optical_flow/rad_native
  -> MicoLink encode -> 27-byte frame -> MicoLink decode
  -> /sim/optical_flow/rad -> sensor pipeline
```

FCU optical-flow routing remains disabled by default.

## Live COM17 Evidence

A checksum-aware 5.002 s capture produced:

| Metric | Result |
|---|---:|
| Serial configuration | 115200 baud, 8N1, no flow control |
| Bytes received | 13,446 |
| Valid frames | 498 |
| Sensor rate | 99.56 Hz |
| Checksum failures | 0 |
| Frame header | `0xEF` |
| Device/system ID | `0x0F / 0x00` |
| Message ID | `0x51` |
| Payload/frame size | 20 / 27 bytes |
| Sensor-time increment | 10 ms |

One verified hardware frame is retained as a unit-test fixture:

```text
EF 0F 00 51 35 14 A0 56 11 00 14 00 00 00
FF 00 01 FF 00 00 00 00 36 01 FF FF E7
```

It decodes as `time_ms=1136288`, `distance_mm=20`, `tof_status=1`,
`flow_velocity=(0,0)`, `flow_quality=54`, and `flow_status=1`.

The subsequent WSL/ROS bridge test decoded 5,519 frames with zero checksum
errors, zero length errors, zero discarded bytes, zero sequence gaps, and zero
sensor-interval repairs. The published sample retained the 10 ms integration
window, 0.020 m range, and measured quality.

## Wire Contract

The implementation follows MicoAir's published MicoLink definition:

```text
header, device_id, system_id, message_id, sequence, payload_length,
payload[20], checksum
```

The checksum is the unsigned 8-bit sum of all bytes before the checksum. The
`0x51` payload is little-endian:

```text
uint32 time_ms
uint32 distance_mm
uint8  strength
uint8  precision
uint8  tof_status
uint8  reserved1
int16  flow_velocity_x
int16  flow_velocity_y
uint8  flow_quality
uint8  flow_status
uint16 reserved2
```

MicoAir defines the flow velocity as `cm/s at 1 m`. The bridge converts it to
an integrated small-angle observation over the sensor-reported interval. Range
is converted from millimetres to metres. Invalid status fields remain invalid;
they are not replaced with plausible values.

## Timing and Rotation

MicoLink carries sensor uptime but no gyro integral. The bridge uses successive
`time_ms` values for the integration duration and stamps the ROS message at
receipt. FCU HIGHRES_IMU samples are integrated over the same arrival-time
window and converted from ROS FLU to sensor FRD axes.

The live protocol test did not run MAVROS, so `integrated_x/y/zgyro` were NaN by
design. This proves the serial path only; it is not a rotation-compensation or
flight-quality result. Full admission to fusion still requires FCU IMU coverage
and the existing yaw-rate reliability gate.

The sensor was stationary during this capture. Frame format and units are
verified, but the installed `flow_velocity_x/y` axis signs are not. Before flight,
move the sensor along positive body X and positive body Y at a measured height,
then confirm the FRD mapping and velocity scale against an independent reference.

## Running the Hardware Bridge

In Windows PowerShell:

```powershell
$Repo = (wsl -d Ubuntu-22.04 -- wslpath -w /home/zyc/multi-slam-github-staging).Trim()
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
& (Join-Path $Repo 'tools\windows_mtf01_serial_bridge.ps1') `
  -Port COM17 -BaudRate 115200 -TcpPort 5764
```

In WSL:

```bash
cd "$HOME/multi-slam-github-staging"
bash tools/run_mtf01_hardware_bridge.sh
```

The WSL output topics are:

- `/hardware/mtf01/micolink_frame`: checksum-validated 27-byte frames.
- `/hardware/mtf01/optical_flow/rad`: estimator-compatible flow observation.
- `/hardware/mtf01/range`: millimetre-quantized range observation.
- `/mtf01/micolink_diagnostics`: rate, checksum, sequence, timing, and IMU coverage.

For sensor-pipeline use, set
`optical_flow_input_topic:=/hardware/mtf01/optical_flow/rad`.

## Deployment Decision

The MTF-01 and MTF-01P both support MicoLink, `mav_apm`, `mav_px4`, MSP, and
AUTO, but they are not always emitting the same protocol. The actual COM17
stream is authoritative for this direct-computer path. Before aircraft use,
set the device explicitly to MicoLink in MicoAssistant and read it back after a
power cycle. Do not leave protocol selection dependent on AUTO detection when
the sensor is connected to the companion computer rather than a flight
controller.

References:

- MicoAir MTF-01 manual: <https://micoair.cn/zh/docs/sensors/sensors/mtf-01-sensors>
- MicoAir MicoLink definition: <https://micoair.cn/zh/docs/sensors/micolink>
- MicoAir assistant guide: <https://micoair.cn/zh/docs/software-tutorial/micoassistant-guide>
