# Reliability Formula Mapping

The implementation follows Ultra-Fusion Section III-D, equations (15) and (18)-(23). A larger `D` means lower reliability. Every weighted sum uses non-negative normalized weights.

## LiDAR: equations (18)-(19)

For each point-to-plane match, the adapter computes `J_i = [n_i^T, -n_i^T [p_i]_x]` and `H_k = sum(J_i^T J_i) + 1e-8 I_6`. It reports the six ordered Hessian eigenvalues, condition number, normal second-moment eigenvalues, a weak-axis proxy and match count. `D_L` is the exact four-term structure in equation (19): Hessian degeneracy, normal diversity, weak-axis penalty and insufficient matches.

The adapter is external to FAST-LIO and estimates planes from consecutive registered scans. Its message therefore sets `approximate=true`; it is not presented as FAST-LIO's internal scan-to-map Hessian.

## Visual/RGB-D: equation (20)

`D_V` uses feature support count, 8x8 spatial occupancy and the equation (20) reprojection term. The current independent RGB-D front end does not yet have calibrated feature reprojection, so that term is emitted as `-1` and excluded from weight normalization. Depth-valid ratio is an explicit project extension. Blur energy is diagnostic evidence and reduces feature support naturally; it is not substituted for reprojection residual.

## IMU: equation (21)

`D_I` uses excitation consistency and the saturation indicator from equation (21). The scoring API accepts the Mahalanobis preintegration residual, but Stage 3 has no unified backend yet, so it emits `-1` and excludes that term. Stage 7 must connect the real preintegration residual without changing the score definition.

## Optical flow: equation (22) adaptation

Ultra-Fusion has no optical-flow-specific equation. `D_OF` adapts the wheel/inertial increment consistency term in equation (22): `min(1, abs(delta_p_OF - delta_p_pred) / tau_v)`. Optical-flow quality and valid ground distance are additional admission terms. No yaw term is claimed because this sensor only supplies horizontal flow in the current simulation.

## BDS/GNSS: equation (23)

`D_B` implements equation (23) with fix quality, covariance trace and Mahalanobis innovation. A BDS outage forces `q_fix=0`; a position jump raises the innovation term. Satellite count and DOP are unavailable in `sensor_msgs/NavSatFix` and are explicitly emitted as unavailable rather than synthesized into the score.
