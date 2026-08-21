# Optical-Flow Lever-Arm Ablation

Both runs used the rich-texture short rectangle, the opt-in APM-equivalent
velocity factor, barometer fallback, and otherwise identical settings.

| Mode | XY RMSE | XY P95 | XY max | Z RMSE | Flow factors | Rollbacks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Lever-arm disabled | 2.042 m | 2.501 m | 2.521 m | 13.938 m | 666 | 0 |
| Lever-arm enabled | 0.659 m | 1.061 m | 1.344 m | 5.751 m | 662 | 0 |

Both missions completed all four waypoints and landed/disarmed. The enabled
case materially improves horizontal error, so IMU-based lever-arm compensation
is retained and remains the default. It does not yet satisfy the absolute
position gate; this experiment only validates the relative improvement.

Evidence:

- `logs/rich_texture_flow_velocity_no_lever_20260821_212526/`
- `logs/rich_texture_flow_velocity_with_lever_20260821_213011/`
