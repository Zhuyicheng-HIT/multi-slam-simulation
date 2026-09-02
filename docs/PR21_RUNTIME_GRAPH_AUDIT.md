# PR21 Runtime Graph Consolidation Audit

Baseline: PR20 `5f588dcbfb8d1f61e3335bd47d6c74457421666f`

## Boundaries

The optimizer, FAST-LIO, relocalization compute node, and ExternalNav safety
gate remain separate processes. They have independent failure handling and
their factor/topic semantics are unchanged. Reliability monitor and scheduler
also remain separate: the scheduler consumes monitor scores, and keeping the
failure domains separate avoids making loss of one diagnostics component take
down the other.

## Production consolidation

`sensor_relay_manager` is one multi-threaded rclpy process that performs only
the existing production relay/unit-normalization copies for active modalities.
It preserves the sensor-data QoS and all public topics. The old per-modality
`fault_injector_*` processes are now test-only and are launched only when
`enable_fault_injection:=true` (or an explicit legacy fault environment is
present). Thus production no longer starts six idle Python injectors.

The body point filter, GNSS metadata association, reliability nodes, LIO
adapter, backend, relocalization, and ExternalNav gate retain their existing
topic ownership because they perform distinct work or are explicit safety
boundaries.

## Profiles

| Profile | Sensor relay modalities | Vision | Fault injector |
|---|---|---|---|
| `minimal_lidar_imu` | LiDAR, IMU | off | off |
| `four_source` | LiDAR, IMU, GNSS, Flow | off | off |
| `five_source` | LiDAR, IMU, GNSS, Flow, RGB-D | on | off |
| `robustness` / `test` | LiDAR, IMU, GNSS, Flow, RGB-D | on | on |

The backend scheduler still receives its established modality names (`vision`
for RGB-D), while the relay manager uses `depth` and `color` only for transport.
Callers can continue using the original topic names and launch files.

## Resource/endpoint expectation

On the production four-source profile, six per-modality relay processes are
replaced by one relay process (a reduction of five Python processes). The DDS
graph removes five duplicate subscription/publisher sets; high-rate topics are
copied once per active modality in the manager. Minimal mode removes the GNSS
association path and Flow relay in addition. Robustness mode intentionally
keeps isolated injectors for fault attribution.

No new high-frequency timer or estimator callback was added. The manager uses
message callbacks only; its two executor threads service independent sensor
callbacks. CPU/RAM and startup wall time must be measured on the target Intel
computer; this change provides the graph reduction but does not claim hardware
measurements from the development host.

## Compatibility and correctness

Raw input topics are untouched. Public normalized topics and QoS remain the
same, IMU acceleration normalization uses the existing SI conversion helper,
and no fusion factor, weight, HXY, or one-observation-one-factor code changed.
The fault-injection launch remains available as an explicit test profile.
