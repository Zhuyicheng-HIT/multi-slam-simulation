# Final Ultra-Fusion visual execution report

## Outcome

Branch `feat/ultrafusion-visual-tight-coupling-v1` is based solely on commit
`57930c86d7d96468b3416f84f8e6f504f527df8a`. It implements a real RGB-D
feature front end, paper-aligned reprojection residuals in the existing
manifold window, vision evidence in the existing FRS, and an opt-in online
source-aware RGB-D/LiDAR map. RTAB local odometry is not used in paper mode;
RTAB remains for persistence, loop closure, cross-session localization and the
legacy A/B factor.

## Verification matrix

| Item | Result | Evidence |
|---|---|---|
| Stable-tag identity | PASS | annotated tag/tree/commit recorded |
| 15-package build | PASS | complete colcon build |
| Colcon tests | PASS | 57 tests, 0 errors/failures |
| New direct tests | PASS | 14 tests |
| Jacobian finite difference | PASS | visual factor test |
| Reliability logic | PASS | tracker/reliability tests and live topics |
| Launch defaults | PASS | both new modules default disabled |
| Lifecycle cleanup | PASS | short lifecycle test and no residual process/port |
| Deterministic ablation, 3 seeds | PASS | `ABLATION_RESULTS.csv` |
| Online shared-map deterministic run | PASS | three stable runs |
| Live RGB-D feature front end | PASS | RGB/depth/CameraInfo/tracks topics observed |
| Live paper visual factor accepted | PARTIAL | unified state was not committed |
| LiDAR/IMU/GNSS/flow live inputs | PASS at transport level | all required topics observed |
| Full small_rectangle/S-curve | PARTIAL | preflight timeout before route start |
| Degraded-scene matrix | BLOCKED | depends on a healthy live unified state |
| Cross-session representative rerun | PARTIAL | tooling migrated; not rerun after runtime gate |

## Runtime root-cause trail

Round 1 found an invalid eight-value camera-rotation parameter and a wrong
CameraInfo topic. Round 2 proved the local FAST-LIO checkout had only the first
downstream patch: importing four stable-tag message types failed as a group,
silently disabling native mode. The pinned second patch was applied to the
external dependency and FAST-LIO rebuilt successfully. Round 3 then proved all
sensor, RGB-D, feature-track, D_V, scheduler and NativeLidarFactor topics were
live and the backend explicitly reported native-factor mode, but it did not
publish `/fusion/unified/odom` within 120 seconds. No `/Odometry` fallback,
threshold relaxation or integrity-check bypass was used.

## Honest conclusions

The new front end is real and produces measured KLT/depth/PnP evidence. The
factor is genuinely wired into the same manifold window and tested there, but
live factor acceptance is not claimed because the native backend did not
commit its first state. D_V is paper-aligned at unit and transport level. The
new online shared map is implemented and deterministic, but real flight map
quality remains unmeasured. This branch is suitable for controlled team
hardware/simulator debugging, not yet for a Draft PR claiming full runtime
completion.
