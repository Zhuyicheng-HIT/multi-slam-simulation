# Rich-Texture Optical-Flow Ablation

All three runs used the same rich-texture `low_indoor_apm_rgbd_mid360` world and
the short rectangle route. The floor visual now uses the existing checkerboard
albedo texture. No estimator tuning was applied.

## Results

| Run | Configuration | Result |
| --- | --- | --- |
| 1 | Unified backend: IMU + optical flow | Mission completed and landed. 491 optical-flow factors accepted. 3D RMSE 1.0108 m, XY RMSE 0.9741 m, Z RMSE 0.2698 m. Accuracy gates failed. |
| 2 | Unified backend: IMU + optical flow + `RANGE_FACET_ENABLED=true` | Mission completed and landed. RangeFacet was genuinely enabled, but only 1 flow-range factor was accepted and 747 were rejected, mainly `nonpositive_intersection`. 79 optimization rollbacks occurred. 3D RMSE 1.8041 m, XY RMSE 1.7920 m, Z RMSE 0.2083 m. This is not a stable direct Z constraint. |
| 3 | APM EKF3: IMU + MAVROS optical flow + range | EKF3 logs explicitly report `fusing optical flow`. Takeoff, all 4 waypoints, landing, and disarm completed. No GPS fusion was observed. The unified backend running in parallel still had poor accuracy (3D RMSE 0.8007 m), so APM stability does not validate the backend. |

## Conclusion

Rich texture materially improves optical-flow admission, but it is not the only
cause of the unified-backend failure. The raw optical-flow path and APM EKF3
fusion are stable in this scene; the remaining defect is downstream in unified
backend flow modeling, gating/rotation handling, and transaction behavior.

The RangeFacet experiment also shows that the current frozen implementation is
a flow-range plane-intersection factor, not an independent constrained world-Z
measurement. It must not be treated as a successful Z-axis test until the
intersection and rollback behavior are corrected.

## Evidence

- `logs/rich_texture_flow_backend_20260821_184000/`
- `logs/rich_texture_flow_rangefacet_valid_20260821_191620/`
- `logs/rich_texture_apm_ekf3_flow_valid_20260821_190938/`
