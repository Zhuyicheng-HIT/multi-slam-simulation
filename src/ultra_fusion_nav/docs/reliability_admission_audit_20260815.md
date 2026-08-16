# Reliability Admission Audit - 2026-08-15

## Scope

This iteration changes admission semantics without changing the factor residuals,
the sliding-window state, or the map representation:

- RGB-D camera motion/parallax is no longer a sensor-health requirement.
- Optical-flow non-zero translation is no longer a sensor-health requirement.
- IMU admission is health-only; low excitation and preintegration NIS remain
  diagnostics and factor-consistency evidence.
- The companion-computer GNSS stream is latest-sample throttled to 2.5 Hz.
- Fixed measured extrinsics remain authoritative. Online extrinsic calibration
  stays optional; online time calibration remains shadow-only.

## Paper Boundary

Ultra-Fusion Eq. (20) scores visual feature support, spatial distribution, and
reprojection residual. It does not include camera-motion parallax in `D_V`.
Parallax appears in the paper's dynamic visual-inertial initialization and
keyframe policy, not camera health.

Ultra-Fusion does not define an optical-flow modality. Its wheel score cannot
be copied mechanically to optical flow. A compensated zero-flow observation
during hover is a valid zero-horizontal-motion observation.

Eq. (21) combines IMU excitation, preintegration consistency, and saturation.
For this UAV implementation, excitation and preintegration NIS are retained as
paper evidence but are separated from hard IMU admission. This is an explicit
engineering deviation: the FCU IMU is mandatory and must not be disabled merely
because the aircraft is hovering.

## Physical Scoring Contract

| Modality | Sensor health / integrity | Factor consistency | Observability | Admission rule |
| --- | --- | --- | --- | --- |
| LiDAR | stream, timestamp, finite points, match support | point-plane residual and innovation | Hessian eigenvalues, condition number, normal/spatial coverage | retain the paper-style geometric score and factor-level robust gate |
| GNSS/BDS | fix type, satellites, DOP, covariance, outage | current prefit NIS using `P_pred + R`, jump gate | absolute position only; no direct yaw claim | direct evidence may bootstrap conservatively; current NIS is authoritative |
| IMU | stream, monotonic timestamp, finite sample, saturation; future validated noise/bias monitors | preintegration NIS, diagnostic and factor-level robust handling | excitation affects calibration/bias observability, not basic admission | enabled unless a direct extreme health fault is present |
| Optical flow | timestamp/integration window, quality, range, gyro compensation | compensated line-of-sight residual against the unified prediction | horizontal relative motion; zero motion is still informative | no non-zero-motion gate; unhealthy packets and uncompensated high yaw remain gated |
| RGB-D vision | synchronized RGB/depth, finite 0.3-6.0 m depth, track support | PnP/reprojection and backend prefit residual | new-keyframe information may depend on motion, camera health does not | no parallax gate; depth/geometric/PnP checks remain |

The scheduler must not sum all modality failures into one global rejection. IMU
is mandatory; auxiliary capability comes from the best currently trustworthy
sources. Each factor still has its own consistency gate.

## Initialization Audit

- Normal takeoff uses a stationary IMU bias estimate (minimum samples and time
  span) before route motion.
- If data are already moving, the estimator can continue and estimate biases in
  the window, but this is a practical fallback, not the paper's complete OAI
  dynamic initialization.
- Visual initialization requires consecutive PnP- and backend-prefit-valid
  batches. It no longer requires a separate motion-parallax health gate.
- Fixed measured extrinsics remove the need to excite online extrinsic
  calibration during every startup.
- Time offset remains unobservable at rest. Online time calibration therefore
  still needs informative rotation/translation, but it runs in shadow mode and
  neither blocks visual factors nor rewrites their timestamps.

## GNSS Algorithm Rate

The FCU/MAVROS source remains untouched. The sensor pipeline publishes the
newest `NavSatFix` and matching `GPSRAW` metadata to the estimator at 2.5 Hz,
preserving source timestamps and dropping superseded samples rather than
replaying a FIFO backlog. A missing metadata match does not suppress a valid
fix; diagnostics report paired and unpaired counts.

Measured source-stamp output rate in the short simulation was 2.499-2.500 Hz.

## Verification

Focused and package-level tests passed before the final flight attempt:

- optical-flow rotation/recovery tests: 8 passed;
- RGB-D frontend focused tests: 14 passed; package tests: 7 passed;
- reliability focused tests (including IMU admission): 59 passed;
- reliability, visual frontend, backend, sensor pipeline, and simulation package
  builds/tests completed with zero reported failures;
- GNSS relay tests cover latest-only behavior, source-stamp preservation,
  missing raw metadata, and low-RTF source-rate evaluation.

The pre-change conservative 2.0 m x 1.2 m single rectangle completed with
ATE RMSE 0.03168 m and translation RPE RMSE 0.00665 m.

The first post-change rectangle is not an accuracy result. A second headless
simulation joined the same ROS domain at route start, producing two interleaved
IMU streams. The scheduler correctly detected non-monotonic timestamps and
entered FAILSAFE. Before that contamination, the run did verify:

- GNSS estimator input rate: 2.499 Hz;
- RGB-D published candidates: 118 instead of the baseline's 1;
- visual factors accepted by the solver: 41/41;
- no visual solver rejection or rollback in the observed segment.

A clean single-instance rectangle must be repeated after the WSL service is
restarted before comparing trajectory accuracy or tuning weights.

## Remaining Gaps

1. Implement validated IMU noise and bias-anomaly monitors from stationary and
   in-flight real FCU data; do not invent thresholds from the ideal simulator.
2. Make optical-flow prefit NIS use the unified prediction and `P + R`, not an
   independent LiDAR odometry truth surrogate.
3. Keep visual time calibration shadow-only until repeated offsets are stable
   and observable; fixed extrinsics remain the primary calibration source.
4. Add launch-time ROS-domain/topic ownership protection so two simulation
   instances cannot silently publish the same estimator inputs.
5. Repeat one clean conservative rectangle, then run isolated factor ablations
   before changing modality weights.
