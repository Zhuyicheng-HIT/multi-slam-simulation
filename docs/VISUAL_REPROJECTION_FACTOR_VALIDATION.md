# Visual reprojection factor validation

## Contract

`VisualFeatureTracks` carries exact current/previous stamps and per-track ID,
normalized and pixel coordinates, metric depth/inverse depth, age, KLT
forward-backward error, geometric-inlier flag and reprojection error. The
backend associates both stamps with adjacent window states after applying the
configured camera time offset. Paper mode enables only this local visual
factor; RTAB odometry remains available solely in the explicit legacy A/B mode.

## Mathematical checks

- The analytic right-local SE(3) Jacobians for both body poses match central
  manifold finite differences in deterministic tests.
- Non-finite observations, non-positive depth/inverse depth, projection behind
  the camera and invalid covariance are rejected before optimization.
- A 2.5-sigma Huber loss bounds individual normalized-image residuals.
- Information is multiplied by `reliability_weight/covariance_inflation` from
  the existing scheduler; no second scheduler exists.
- Admission diagnostics distinguish timestamp/window mismatch, insufficient
  depth-valid KLT/geometric tracks, scheduler disable and geometric failure.

## Results

All four factor tests passed, including finite-difference Jacobians, invalid
input rejection and pose correction. In the three-seed synthetic A/B harness,
paper reprojection achieved mean translation RMSE 0.002061 m and rotation RMSE
0.000561 rad, versus 0.024529 m/0.019108 rad for the four-source factor set and
0.010783 m/0.005669 rad for the legacy RTAB-style relative factor. These are
factor-level deterministic regressions, not flight or real-world accuracy.

The live front end published `/vision/feature_tracks` in the final headless
attempt, but `/fusion/unified/odom` did not reach its first committed state.
Consequently, nonzero live visual-factor acceptance is PARTIAL and is not
inferred from topic presence.
