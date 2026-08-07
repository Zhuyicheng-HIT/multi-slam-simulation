# Ultra-Fusion paper gap analysis

## Audited baseline

The sole code baseline is annotated tag
`v0.1.0-four-source-reloc-calibration`: tag object
`4325b5bdcbe588110039a1b578e282a9e7d03c89`, dereferenced commit
`57930c86d7d96468b3416f84f8e6f504f527df8a`, tree
`1eca12f9266ad0e318d6ffa7453109ea01547256`. The newer Stage3 tip was
audited but not merged.

| Capability | Stable tag | This branch | Classification |
|---|---|---|---|
| SO(3) sliding-window state `R,p,v,b_a,b_g` | implemented | unchanged | PASS, compact reproduction |
| Native LiDAR point-to-plane factor | implemented | unchanged | PASS |
| IMU preintegration | implemented | unchanged | PASS |
| GNSS | ENU position anchor | unchanged | PARTIAL: not paper pseudorange/Doppler |
| Optical flow | body displacement constraint | unchanged | PARTIAL: paper adaptation |
| FRS/OAI | scheduler and observability gates | vision added to same FRS | PARTIAL reproduction |
| LiDAR-IMU calibration | shadow-only | unchanged | PARTIAL; application remains locked |
| Visual reprojection | absent | inverse-depth two-state factor | PASS for V1 geometry |
| Optimized visual landmarks | absent | fixed measured RGB-D depth | PARTIAL |
| Camera time offset | absent | fixed correction interface | PARTIAL; not estimated online |
| Camera-IMU extrinsic | absent | measured parameter interface | PARTIAL; not estimated online |
| Relocalization | LiDAR workflow | RTAB persistence workflow retained | PASS tooling; current runtime rerun PARTIAL |
| Geometric/color map | FAST-LIO geometry | source-aware RGB-D/LiDAR voxel map | PASS deterministic V1 |

The visual residual follows the paper's two-frame inverse-depth geometry:
`p_Ci = [x_i,y_i,1]^T/rho_i`,
`p_Cj = T_BC^-1 T_WBj^-1 T_WBi T_BC p_Ci`, and
`r = pi(p_Cj)-[x_j,y_j]^T`. Right-local analytic pose Jacobians, normalized
pixel covariance, inverse-depth uncertainty, Huber loss and FRS information
scaling are implemented. Depth is deliberately not claimed as an optimized
landmark.

## Related-system design review

FAST-LIVO2 and R3LIVE informed the single geometric/color-map ownership idea;
LVI-SAM informed modular factor ownership; VINS-RGBD informed depth-aided
inverse-depth initialization; Ground-Fusion and LIC-Fusion informed explicit
spatiotemporal calibration gates. No external source code was copied. Licenses
were reviewed: FAST-LIVO2/R3LIVE GPL-2.0, LVI-SAM BSD-3-Clause,
VINS-RGBD/Ground-Fusion GPL-3.0.

## Remaining paper gaps

- No raw GNSS pseudorange/Doppler model.
- No joint optimization of inverse-depth landmarks.
- No online observable camera extrinsic/time-offset estimator.
- No full cross-factor covariance propagation.
- The shared-map conflict rule is an engineering V1; the paper does not define
  this exact RGB-D voxel policy.
- The final headless flight could not commit its first native-triggered state
  after three repair/verification rounds, so flight-level visual acceptance,
  ATE/RPE and degraded-scene results remain PARTIAL rather than fabricated.
