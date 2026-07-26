# Reliability Formula Mapping

The implementation follows Ultra-Fusion Section III-D, equations (15) and (18)-(23). A larger `D` means lower reliability. Every weighted sum uses non-negative normalized weights.

## Missing-evidence policy

Paper weights keep their original denominator. If a paper term is unavailable, its value is not guessed and the remaining terms are not renormalized upward. Every score publishes `evidence_weight_coverage` and `score_complete` in its evidence arrays. An incomplete score remains useful for diagnostics and monotonic fault tests, but the ROS message sets `valid=false` and `reliability_weight=0` so it cannot accidentally become a scheduler factor weight.

LiDAR factor scoring has one deliberate exception: complete equation-(19) geometry may retain a conservative continuous weight while the LiDAR-free innovation is temporarily unavailable. That result carries `hard_gate_allowed=0`; it can inflate covariance but cannot authorize a binary LiDAR-factor shutdown. This avoids treating an external approximate Hessian as proof that the current LIO pose factor is wrong.

`ReliabilityScore.reliability_weight` is only a guarded Stage 3 inverse-score diagnostic. Equation (15), hysteresis, covariance inflation, and the final factor weight belong to Stage 6 `ReliabilityScheduler`; Stage 3 does not claim that `1-D` is the paper scheduler.

## LiDAR: equations (18)-(19)

For each point-to-plane match, the adapter computes `J_i = [n_i^T, -n_i^T [p_i]_x]` and `H_k = sum(J_i^T J_i) + 1e-8 I_6`. It reports the six ordered Hessian eigenvalues, condition number, normal second-moment eigenvalues, a weak-axis proxy and match count. `paper_score_eq19` preserves the exact four-term structure in equation (19): Hessian degeneracy, normal diversity, weak-axis penalty and insufficient matches.

The implementation publishes two different project scores instead of mixing their evidence:

- `/reliability/lidar_score` is pose-factor risk. It combines equation-(19) geometry with the innovation between the current LIO pose and a prediction formed before the current LiDAR factor. With the current external geometry adapter, geometry contributes only `0.20`; innovation contributes `0.80` when available. `hard_gate_allowed=0` while either geometry is approximate or innovation is incomplete. Thus the score can continuously change a factor weight and covariance but cannot by itself switch LiDAR off.
- `/reliability/lidar_map_score` is map-admission risk. It contains residual P95, spatial coverage, dynamic ratio, uncertain ratio, and feature repeatability. It does not feed the LiDAR pose-factor gate. `map_quality` is diagnostic-only because the adapter derives it from dynamic ratio and repeatability, so including it in the weighted map score would double-count those signals.

The geometry adapter is external to FAST-LIO and estimates planes from consecutive registered scans. It therefore sets `approximate=true`; it is not presented as FAST-LIO's internal scan-to-map Hessian. A native point-to-plane residual/Hessian export is required before hard LiDAR gating or a full tightly coupled claim.

## Visual/RGB-D: equation (20)

`D_V` uses feature support count, 8x8 spatial occupancy and the equation (20) reprojection term. The current independent RGB-D front end does not yet have calibrated feature reprojection, so that term is emitted as `-1`; its paper weight remains reserved and evidence coverage is `0.75`. Depth-valid ratio is an explicit project extension. Blur energy is diagnostic evidence and reduces feature support naturally; it is not substituted for reprojection residual.

## IMU: equation (21)

`D_I` uses excitation consistency and the saturation indicator from equation (21). The scoring API accepts the Mahalanobis preintegration residual, but Stage 3 has no unified backend yet, so it emits `-1`, reserves that weight, and reports evidence coverage `0.55`. Low excitation is labeled as an observability risk, not an IMU hardware fault. Stage 7 must connect the real preintegration residual without changing the score definition.

## Optical flow: equation (22) adaptation

Ultra-Fusion has no optical-flow-specific equation. `D_OF` adapts the wheel/inertial increment consistency term in equation (22). The paper uses a vector increment residual; the current simulator has not yet validated the optical-frame transform and scale against an estimator-side prediction. Stage 3 therefore marks the increment term unavailable instead of comparing against the zero `Odometry.twist` field. Optical-flow quality and valid ground distance remain project-specific admission evidence, with coverage `0.40` until Stage 4 provides a calibrated prediction. No yaw term is claimed.

## BDS/GNSS: equation (23)

`D_B` implements equation (23) with fix quality, covariance trace and an external LIO-consistency innovation proxy. A BDS outage forces `q_fix=0`; a position jump raises the innovation term. The current proxy compares GNSS and LIO displacement magnitudes rather than the full local-frame innovation vector, so reports and reasons keep that limitation explicit. Satellite count and DOP are unavailable in `sensor_msgs/NavSatFix` and are emitted as unavailable project extensions rather than synthesized into the paper score.
