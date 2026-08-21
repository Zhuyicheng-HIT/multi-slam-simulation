# APM-Equivalent Optical-Flow Factor Ablation

This change adds an opt-in `optical_flow_velocity_factor_enabled` mode. It
keeps the frozen displacement factor as the default and adds a horizontal body
velocity factor using the APM-compensated flow measurement and exposure time.
RangeFacet and LIO increment consistency are bypassed in this mode; basic flow
quality, range, timestamp, and IMU coverage checks remain active.

## Short-rectangle results

| Configuration | XY RMSE | Z RMSE | Flow factors | Rollbacks | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| Rich texture, displacement factor | 0.974 m | 0.270 m | 491 | 0 | Accuracy failed |
| Rich texture, velocity factor | 0.886 m | 9.23 m | 684 | 0 | Accuracy failed |
| Velocity factor + barometer fallback | 0.637 m | 7.83 m | 433 | 0 | Accuracy failed |

All missions completed the four waypoints and landed/disarmed. The velocity
factor removed the previous transaction instability and improved horizontal
error, but it does not create absolute XY position. Without GNSS, LiDAR, or
vision position information, this test cannot meet an absolute trajectory
accuracy gate. The barometer result confirms that it helps vertical
observability but does not solve horizontal drift.

The new mode remains opt-in until a velocity-factor test with an independent
XY source and a relative-drift metric is passed.

Evidence:

- `logs/rich_texture_flow_velocity_apm_20260821_193902/`
- `logs/rich_texture_flow_velocity_apm_barometer_20260821_194520/`
