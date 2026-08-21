# Resilient Navigation Goal and Paper Baseline - 2026-08-20

## Objective

The target is navigation with a non-severely-degraded FCU IMU and at least one
other trustworthy positioning modality. A degraded modality must not dominate
the common state, map, marginalization prior, or ExternalNav output.

This is stronger than proving that the full five-source configuration works.
It requires evidence that every supported aiding source is independently
useful with IMU, and that isolated and concurrent degradation is attributed to
the faulty modality before it corrupts the estimator.

Gazebo truth is evaluator-only. It must never enter factor admission, state
estimation, route feedback, relocalization acceptance, or map maintenance.

## Paper Baseline

The primary reference is Ultra-Fusion, arXiv:2606.21223. The local audit uses
`/home/ld666/ultrafusion/Ultra-Fusion.pdf`.

The paper's relevant algorithm boundary is:

1. Heterogeneous observations are timestamp-ordered and converted to optional
   factors in one shared sliding-window estimator. LiDAR point-to-plane
   residuals remain in this common estimator instead of entering as an
   independently optimized LiDAR-odometry posterior.
2. At every keyframe, each modality receives its own degradation score
   `D_k^(m)`. Equation (15) maps this evidence and a minimum observation count
   to a modality activation variable `s_k^(m)`.
3. Equation (16) uses `s_k'^(m) L_m(k')` for every factor timestamp `k'` in the
   active window. Reliability is therefore factor-time-specific; the current
   state must not blindly overwrite every historical factor weight.
4. Short-horizon hysteresis prevents rapid factor switching. The paper allows
   deactivation and covariance inflation/down-weighting before backend
   optimization.
5. LiDAR degradation evidence is based on point-to-plane Hessian conditioning,
   normal diversity, weak-axis support, and match count. Visual evidence uses
   support, spatial distribution, and reprojection consistency. IMU evidence
   uses excitation, preintegration consistency, and saturation. GNSS evidence
   uses fix integrity, covariance, and local innovation consistency.
6. Historical information is retained through a Gaussian marginalization
   prior. The paper does not specify retrospective removal of one modality from
   an already marginalized prior.
7. Observability-Aware Initialization is a startup model-selection mechanism.
   Online Spatiotemporal Calibration is enabled only under sufficient
   reliability and excitation. Neither mechanism replaces runtime factor
   reliability scheduling.
8. Optional intensity consistency supplements LiDAR geometry when local
   support exists. It is not a substitute for suppressing ill-conditioned
   geometric factors.

The paper reports that LiDAR FRS lowers mean ATE by 75.3% in its ablation. Its
LiDAR-degenerate simulation results for Ultra-Fusion without/with intensity
are 0.10/0.06 m on Wild01, 0.95/0.70 m on Wild02, 0.08/0.08 m on Tunnel01, and
2.07/2.21 m on Tunnel02. These are external reference results, not acceptance
thresholds that this repository has already reproduced. The estimator source
is not public, so implementation details absent from the paper must be treated
as engineering extensions rather than attributed to Ultra-Fusion.

## Required Invariants

- The unified window owns the final state.
- One physical observation enters one optimization transaction at most once.
- Every factor stores its measurement timestamp, evidence timestamp,
  reliability decision, covariance inflation, and admission reason.
- Sensor health, geometric observability, and cross-modal consistency remain
  separate evidence channels.
- Disagreement with an already drifted state is not sufficient evidence that a
  directly healthy sensor is faulty.
- Degraded scans cannot irreversibly update the navigation map before their
  map-admission decision is known.
- The FCU primary IMU remains `/mavros/imu/data_raw`.
- Optical flow is horizontal motion only; its range supplies scale and is not a
  world-Z factor.
- Online extrinsics remain subordinate to measured fixed extrinsics. Online
  time calibration remains diagnostic until separately validated.

## Phase 1: Healthy Single-Aiding Validation

Each row must run with the FCU IMU and exactly one aiding modality contributing
navigation factors. Other streams may be recorded for evaluation but must have
zero accepted factors. A full five-source run cannot substitute for these
ablations.

| Configuration | Required world/trajectory evidence | Required checks |
| --- | --- | --- |
| IMU + LiDAR | Planes and edges in all translation axes, turns, height changes, and looped motion | Hessian spectrum, accepted correspondences, map growth, causal pose error, no pose-factor duplication |
| IMU + GNSS/BDS | Outdoor/open-sky route with horizontal and vertical motion | fix metadata, covariance, innovation, accepted fixes, local-frame transform, causal pose error |
| IMU + optical flow | Textured ground, valid range, horizontal translations and yaw segments | quality, compensated flow, range age, horizontal scale/direction, zero Z information |
| IMU + RGB-D | Textured non-repeating geometry, valid depth across 0.3-6 m, translation and rotation | synchronized bundles, feature distribution, reprojection/direct residual, accepted batches, no duplicate visual/depth factor |

For every row, report causal 3-D/XY/Z RMSE, P95 and maximum error; attitude
error; factor received/selected/accepted/rejected counts; output age; solver and
callback latency; RTF; CPU and memory; takeoff, route, landing and disarm; and
whether EKF3 consumed ExternalNav. Accuracy thresholds must be frozen before
fault testing from repeated healthy runs rather than selected after observing a
degradation result.

## Phase 2: Paper-Style Reliability Scheduling

First reproduce the paper's factor-time-specific FRS boundary without adding
retrospective factor rewriting:

- compute complete modality evidence at the factor timestamp;
- apply continuous covariance inflation and binary admission with hysteresis;
- keep the decision attached to that factor while it remains in the window;
- prevent failed front-end/map observations from bypassing backend admission;
- preserve healthy modalities during cross-modal disagreement;
- expose the complete decision timeline and reason codes.

The minimum degradation matrix is:

| Case | Required surviving navigation support |
| --- | --- |
| LiDAR degeneracy only | IMU plus at least one of GNSS, RGB-D, or optical flow |
| GNSS denial/jump only | IMU plus at least one local modality |
| LiDAR degeneracy + GNSS denial | IMU plus healthy RGB-D and/or optical flow |
| Dynamic LiDAR foreground | Static LiDAR support if sufficient, otherwise another healthy modality |
| Visual degradation | IMU plus LiDAR, GNSS, or optical flow |
| Optical-flow degradation | IMU plus LiDAR, GNSS, or RGB-D |

LiDAR tests must include a long corridor/tunnel and a non-degenerate control
world. GNSS tests must include denial and corrupted-but-present measurements.
Simultaneous LiDAR and GNSS degradation is mandatory. Every fault run requires
an identical no-fault control and a fixed-weight or FRS-disabled ablation.

## Phase 3: Bounded Retrospective Revision

Only after Phases 1 and 2 pass, evaluate delayed-detection correction as an
explicit extension beyond the paper:

- retain a short raw-factor/evidence history;
- estimate a degradation onset time from stored evidence;
- revise only non-marginalized factors in the confirmed degradation interval,
  not every past factor of the modality;
- compare mutable provisional factors with immutable per-timestamp FRS in a
  single-variable A/B;
- if contaminated information has already entered the prior, rebuild from a
  healthy checkpoint and retained raw factors rather than algebraically
  subtracting a modality from a mixed Schur prior.

No retrospective policy may be promoted unless it improves fault-onset error
without regressing healthy scenes, recovery, runtime, or factor accounting.

## Current Evidence Status

As of commit `a37539a`, none of the three phases is complete:

- Historical five-source and four-source runs do not prove each IMU-plus-one
  configuration independently.
- Existing worlds include simple, low indoor, city, warehouse, and large
  tunnel assets, but they have not yet passed a common feature-rich world audit
  for every single-aiding configuration.
- The current window stores scheduler weight and covariance inflation when a
  factor is appended. This matches the per-factor indexing of equation (16),
  but delayed-detection retrospective revision is not implemented.
- The static long-tunnel run has already contradicted the navigation target:
  degenerate LiDAR contributed to divergence while healthy GNSS was later
  down-weighted as inconsistent.
- Two local, uncommitted files contain a candidate correction to LiDAR
  axis-information semantics. Unit tests passed previously, but the full replay
  was interrupted and is not valid evidence. These changes are not part of the
  remote baseline.

The next gate is therefore Phase 1 inventory and execution, beginning with
repeatable healthy IMU-plus-one runs before any scheduler threshold or source
factor is tuned.

## Tunnel Evidence Correction - 2026-08-21

The replay accuracy reports derived from the first large-tunnel bags are not
valid absolute-accuracy evidence. The recorded
`/sim/mid360/ground_truth_odom` remained at the origin for every inspected
sample. The direct MID360 bridge had subscribed to pose and world-statistics
topics for its default small world because `run_apm_sensor_stack.sh` did not
pass the selected large-scene Gazebo world name. It then published the
LaserScan sensor `world_pose` as a fallback; that pose was static and is not
the aircraft model pose.

Consequences:

- all tunnel replay RMSE/P95/maximum values computed against that topic are
  withdrawn as navigation-accuracy results;
- the replay runs remain useful only for factor accounting, convergence,
  rollback, queue, and timing comparisons;
- the original online observer CSV remains valid because it queried the
  Gazebo `apm_iris` model pose directly;
- GNSS ENU axes were checked independently against the overlapping valid
  online truth segment. X/Y/Z motion correlations were 0.969/0.941/0.996, so
  there is no evidence of an ENU axis reversal or sign error. The compared
  samples came from different runs and are not a GNSS accuracy benchmark.

The simulation bridge now receives the actual world and model names, derives
the statistics topic from that world, and fails closed when the model pose is
unavailable. A headless tunnel smoke test produced 180 model-pose truth
updates, zero scan-pose fallbacks, zero unavailable truth samples, and a valid
RTF stream. The body-centered 0.50 x 0.50 x 0.10 m exclusion box was also
verified: approximately 19,840 valid points were retained per scan and only
20-25 points (about 0.11%) were removed. The earlier claim of a 95% body-filter
removal rate was a log-field interpretation error and is withdrawn.

## Paper Backend Configuration - 2026-08-21

The explicit Table XIII configuration has been applied to the manifold
backend: a 10-state window, at most eight LM iterations, and a cooperative
40 ms solve budget. The budget is checked between complete linearization,
cost, and linear-solve kernels; a single kernel cannot be interrupted, so a
small overrun remains possible. Normal convergence still stops before the
iteration cap, and optional IMU re-integration cannot start after the main
solve consumes the budget.

This does not make the implementation identical to the unpublished reference
estimator. The repository still uses its own NumPy/C++-assisted LM solver
rather than Ceres, and equation (19) does not define the weak-axis penalty
function `phi_a` precisely enough to reproduce it uniquely. These limitations
must remain explicit in every comparison.

## Phase 1 Baseline Audit - 2026-08-20

A detached clean worktree at commit `a37539a` was built independently from the
two uncommitted LiDAR Schur-semantics changes in the main worktree. The build of
the backend, reliability, visual, and sensor-pipeline dependency closure passed.
Package tests also passed:

| Package | Passed tests |
| --- | ---: |
| `uf_backend_fusion` | 286 |
| `uf_reliability` | 81 |
| `uf_sensor_pipeline` | 37 |
| `uf_visual_frontend` | 13 |
| Total | 417 |

These tests prove factor algebra and component behavior only. They do not prove
an IMU-plus-one flight. In particular, the scheduler test named
`test_each_single_healthy_source_keeps_output_available` exercises the
scheduler state machine, not backend initialization, trajectory accuracy, or
ExternalNav consumption.

The online backend currently cannot provide a strict non-LiDAR single-aiding
test:

1. In native-factor trigger mode it waits for a valid initial native LiDAR
   factor before creating the first state.
2. The first state position and orientation are copied from the native
   FAST-LIO frame and receive a `1e-4` pose prior even if the scheduler later
   disables the LiDAR factor.
3. Auxiliary IMU-timed keyframes require an already initialized backend and a
   previous LiDAR timestamp, so they handle an outage after startup but cannot
   cold-start IMU plus GNSS, optical flow, or RGB-D.
4. The validation checker currently requires all four legacy factor counters
   to be nonzero. LiDAR's counter includes a factor record even when that record
   is disabled, whereas GNSS, flow, and visual counters represent admitted
   factors. The existing gate therefore cannot prove exact single-modality
   participation.

The first simulation pass will consequently be labeled a transitional seeded
ablation: FAST-LIO may define the initial local gauge, but its subsequent
factor weight is zero. It can validate whether each aiding factor sustains the
trajectory after startup, but it cannot satisfy the final source-independent
single-sensor acceptance requirement. A source-independent OAI/bootstrap and
sensor-neutral keyframe clock are required before Phase 1 can be closed.

## Current Frozen-Baseline Audit - 2026-08-22

The following evidence is from branch `feat/core-algorithm-cleanup-20260817`.
The current remote HEAD is `2721492`; the worktree is clean. These results keep
the original acceptance thresholds and do not use Gazebo truth in estimation.

### Passed Simulation Gates

| Case | 3-D RMSE | 3-D P95 | Maximum | Decision |
| --- | ---: | ---: | ---: | --- |
| MicoLink five-source rich-texture rectangle | 2.92 cm | 3.83 cm | 4.32 cm | pass |
| GNSS outage | 2.98 cm | 3.91 cm | 4.48 cm | accuracy pass; one integrity gate failed |
| LiDAR 75% sparse degradation | 3.05 cm | 4.02 cm | 4.57 cm | pass |
| GNSS + LiDAR dual degradation | 2.95 cm | 3.87 cm | 4.54 cm | pass |
| Relocalization short rectangle | 3.24 cm | 4.36 cm | 5.13 cm | pass |
| Dynamic city rectangle | 16.68 cm | 33.45 cm | 37.83 cm | fail 30 cm P95/max gate |

Relocalization logs show automatic loop transactions, accepted candidates, a
manual candidate acceptance, and two backend window resets. Dynamic-map logs
show historical voxel removal from `/cloud_registered` output and separate
static/dynamic/uncertain visualization topics. Current-frame FAST-LIO native
point-level dynamic filtering is not implemented; the dynamic result must not
be advertised as a full dynamic-object solution.

### Paper-Dataset Evidence

The M2DGR MCAP replay now terminates without the finite-rosbag `/clock` false
positive. The dataset adapter was corrected after inspecting the serialized
cloud: the 16 x N storage tiling is not 16 rings, and per-row time reset was
causing FAST-LIO to deskew one scan repeatedly. The adapter now publishes a
flat height-1 stream, assigns one monotonic relative time over the whole scan,
and restores the interleaved 16-channel ring sequence. At
`PLAYBACK_RATE=0.5`, this improved the best recent result to 5.41 m 3-D RMSE,
12.15 m P95, and 20.13 m maximum error. Native LiDAR factors increased from
24 to 82 and prediction-gate rejections fell from 79 to 15. The result is
still not a pass: the trajectory drifts after the first several seconds and
the backend reports nonmonotonic auxiliary/LiDAR state timestamps near the
end. The adapter lives in the external WSL dataset workspace
`/home/ld666/ultrafusion-datasets/adapters_ws`, which is not a Git checkout;
the replay artifact is
`/home/ld666/ultrafusion-datasets/reports/m2dgr_plus_flat_cloud_20260822_013617`.
Increasing point density (`point_filter_num=1`) worsened the result to
92.03 m RMSE and was reverted.

MARS currently has approximately 10.98 m 3-D RMSE and 10.94 m Z RMSE. R3LIVE
has no valid trajectory association. The detailed artifacts are kept under
`/home/ld666/ultrafusion-datasets/reports/` and summarized in
`/home/ld666/ultrafusion-datasets/reports/dataset_summary_current_20260822.md`.

### Implementation Status

- MicoLink `0x51` companion optical-flow framing is enabled in the simulation
  chain and covered by protocol tests.
- The unified backend, per-factor reliability decisions, loop/relocalization
  components, historical dynamic-voxel removal, and visualization topics are
  present and tested.
- The full objective is not yet accepted: M2DGR/MARS/R3LIVE do not provide a
  passing paper-dataset score, and the dynamic-scene P95/max gate is still
  above 30 cm.
- The next engineering action is dataset-specific point-cloud/FAST-LIO
  geometry compatibility work. Backend gates must remain unchanged until that
  input contract is corrected and independently verified.

### Transitional IMU + GNSS run

Run directory:
`logs/single_aiding_gnss_20260820_first`

The run used fixed reliability weights `IMU=1`, `GNSS=1`, and zero for LiDAR,
optical flow, and vision. It completed takeoff, all four rectangle legs,
landing, and FCU disarm. ExternalNav-to-EKF3 was intentionally disabled, and
the route used FCU local feedback, so the result is estimator-only.

Strict accepted-factor accounting was:

- IMU: 846;
- GNSS: 420;
- LiDAR: 0 accepted from 847 disabled records;
- optical flow: 0;
- vision: 0.

The causal 3-D RMSE/P95/maximum were 0.0735/0.0976/0.2791 m. Horizontal RMSE
was 0.0283 m, vertical RMSE was 0.0679 m, and endpoint error was 0.0542 m. The
maximum error was almost entirely vertical and the longest contiguous 0.20 m
threshold exceedance was 1.386 s. The run therefore failed the frozen 0.20 m
maximum/sustained-error gate despite passing RMSE, P95, endpoint, factor
isolation, route, timestamp, and optimizer-integrity gates.

The output rate was 10.00 Hz with 0.067 s source-age P95 and no duplicate,
regressing, zero, or stale timestamps. Optimizer phase P50/P95/max was
1.74/20.24/36.93 ms. There were no optimization errors or rollbacks.

This run had poor wall-clock efficiency: 83.36 s of simulation required about
410 s wall time, for an approximate RTF of 0.20. Resource sampling reported
backend CPU P50/P95 63.8/72.0%, Gazebo 221.2/236.2%, and MAVROS
160.9/172.9%. Subsequent single-aiding comparisons must use the same observer
load until the resource campaign is separately optimized.

### Transitional IMU + LiDAR run

Run directory:
`logs/single_aiding_lidar_20260820_first`

Strict accepted-factor accounting was IMU 845, LiDAR 846, and zero GNSS,
optical-flow, and vision factors. The run completed takeoff, route, landing,
and disarm. Causal 3-D RMSE/P95/maximum were 0.0284/0.0384/0.0487 m;
horizontal and vertical RMSE were 0.0184 and 0.0216 m. There were no optimizer
errors, rollbacks, or uncommitted native frames. This seeded single-aiding run
passed every configured strict gate.

The route nevertheless exposed a validation-lifecycle bug: after landing, the
rectangle child waited for a fixed 150 s optical-flow truth observer while the
entire simulation remained alive. The observer is now finalized when the route
terminates, and non-flow single-aiding profiles no longer start it.

### Transitional IMU + optical-flow run

Run directory:
`logs/single_aiding_optical_flow_20260820_first`

Strict accepted-factor accounting was IMU 842, optical flow 233, and zero
LiDAR, GNSS, and vision factors. The vehicle completed the FCU-driven route and
landing, but the estimator failed severely: causal 3-D RMSE/P95/maximum were
66.06/181.26/251.38 m, horizontal RMSE was 66.05 m, vertical RMSE was 0.70 m,
and endpoint error was 251.38 m. An optimization rollback and an uncommitted
native trigger were observed.

This is evidence of unified-backend failure under relative horizontal aiding,
not a claim that the optical-flow sensor alone supplies globally bounded 3-D
position. The optical-flow factor correctly has no Z row. The magnitude and
growth of the horizontal failure require backend state/gauge/prior correction
before source-factor tuning.

### Transitional IMU + RGB-D direct run

Run directory:
`logs/single_aiding_vision_rgbd_direct_20260821_second`

The first RGB-D attempt was invalid because excessive observer load reduced
the wall/source ratio to about 0.07 and MAVROS disconnected during the
post-takeoff hold. The valid second run disabled the duplicate SLAM-drift
collector and sampled process resources every 2 s. It completed takeoff, all
four rectangle legs, landing, and FCU disarm. The route used FCU-local feedback
and ArduPilot GPS; GNSS was not admitted to the unified backend and ExternalNav
was not consumed by EKF3.

Strict accepted-factor accounting was IMU 305, RGB-D direct 80, and zero
LiDAR, GNSS, and optical-flow factors. Causal 3-D RMSE/P95/maximum were
0.313/0.524/1.014 m. Horizontal RMSE was 0.261 m, vertical RMSE was 0.173 m,
and endpoint error was 1.014 m. The maximum contiguous 0.20 m exceedance was
4.42 s. The unified output rate was 8.11 Hz, with 0.266 s source-age P95,
1.452 s maximum gap, and 17 stale samples.

The failure occurred after visual factors were accepted, not before sensor
admission. The final backend summary reported 80 visual factors, 339
`ill_conditioned_latest_information` optimization rejections and rollbacks,
339 native triggers consumed without state commit, and 241 superseded native
queue entries. Solve mean/maximum were 28.1/173.7 ms; prepare mean/maximum were
63.0/859.7 ms. The direct factor's latest batch retained 29 tracks and was
photometrically down-weighted, with 0.033 m depth RMSE and 2.27 photometric
RMSE.

The streamlined run advanced 83.97 s of simulation in 135.84 s of runtime
collection, an effective RTF of about 0.62. Preflight wall/source ratios were
0.56 for unified odometry, 0.56-0.60 for RGB-D, and 0.60 for FCU IMU. This is
roughly nine times the invalid first attempt. Median CPU use was 258% for
Gazebo, 233% for MAVROS, 111% for the backend, 84% for vision, and 55% for
FAST-LIO; percentages may exceed 100% because processes use multiple cores.

Together with the GNSS and optical-flow runs, this result changes the immediate
priority from further scenario expansion to the shared backend. In particular,
the transaction integrity gate currently treats weakly observable directions
in the full latest-state information matrix as grounds to roll back an
otherwise finite, cost-reducing update. Paper-style observability-aware
initialization and factor-time-specific reliability must be corrected before
any source threshold is tuned.

## Paper-first backend correction screening - 2026-08-21

The first correction pass changed backend semantics, not sensor weights:

- factor admission now uses the latest scheduler and modality score whose
  timestamp is not later than the factor observation;
- a scheduler-disabled LiDAR factor is no longer restored to a nonzero floor
  unless the explicit frozen `preserve_lio_anchor` option is enabled;
- a non-initial keyframe with no enabled observation factor is transactionally
  deferred rather than optimized and rolled back as an empty state;
- historical factors retain the decision recorded at their own timestamp, as
  required by the per-keyframe activation variable in equation (16).

The frozen tunnel replay was repeated at rate 0.5 with the prior Z-only axis
handoff configuration. The historical A run had 3-D RMSE/P95/maximum
3.543/7.933/9.212 m and 26 rollbacks. The corrected replay in
`logs/tunnel_backend_replay_paper_frs_20260821` produced
3.693/8.214/12.368 m, 24 rollbacks, and five intentionally deferred empty
keyframes. It therefore does not establish an accuracy improvement. The
latest-only worker makes exact effect sizes timing-sensitive, but the maximum
error regression is sufficient to reject promotion of this pass as a stable
navigation baseline.

A subsequent single-variable candidate mapped the weakest per-axis native
LiDAR support directly to the whole-factor weight. It correctly disabled 45
tunnel frames, but also down-weighted 384 partially observable frames. The run
in `logs/tunnel_backend_replay_eq19_frame_frs_20260821` regressed to 3-D
RMSE/P95/maximum 9.612/17.165/63.418 m with 39 rollbacks. The source change was
immediately reverted and the failed evidence retained. Equation (19)'s axis
penalty cannot be implemented as `min(axis support)` on the complete factor;
future work must preserve the observable subspace or require confirmed
modality-wide degradation before disabling the entire LiDAR factor.

The audit also confirmed that the runtime still does not implement the full
Observability-Aware Initialization in paper Algorithm 1. The current first
state is normally triggered and posed by FAST-LIO and receives a strong pose
prior. A pure, unit-tested selector now encodes the paper's exact D, S, M, A
priority and MCC admission boundary. This is only the model-selection boundary:
the dynamic SfM/visual-inertial alignment, source-independent stationary IMU
cold start, and LiDAR short-window MAP branches still need runtime integration.
Until those branches and a modality-neutral keyframe clock are validated, the
single-aiding runs remain seeded ablations rather than strict sensor-only
proofs.

## Observer-load A/B - 2026-08-21

`external_nav_accuracy` associates truth and estimates exclusively by message
header timestamps. Its ROS clock was used only to schedule periodic report
writes, so validation and replay launchers now run this observer on wall time
and avoid a high-rate `/clock` subscription. In an otherwise identical tunnel
replay, its process CPU snapshot fell from about 38% to 17%; rosbag playback
fell from about 56% to 24%. The light-observer run at
`logs/tunnel_backend_replay_paper_frs_light_observer_20260821` produced 3-D
RMSE 3.654 m versus 3.693 m and committed 436 states in both runs. This is
consistent with replay timing variation and shows no metric-contract change.

The validation MAVROS plugin allowlist was also exercised through a complete
RGB-D takeoff, route, landing, and disarm. MAVROS median CPU fell from about
233% to 70%. Resource sampling is now 2 s for single-aiding runs, duplicate
SLAM-drift and FAST-LIO accuracy observers are disabled there, and all
collectors terminate when landing/disarm completes. These changes reduce
online simulation load without changing estimator inputs or factor settings.

## Unified-backend paper correction A/B - 2026-08-21

The completed single-aiding runs all used the unified window and retained the
FAST-LIO-seeded initial gauge; they are factor-isolation ablations, not pure
standalone sensor estimators. LiDAR-only passed with 0.028 m causal 3-D RMSE.
GNSS-only reached 0.074 m RMSE but missed the maximum-error gate at 0.279 m.
Optical-flow-only diverged to 66.1 m RMSE, and RGB-D-direct-only reached
0.313 m RMSE with 339 transaction rollbacks. These results support the shared
backend diagnosis and do not justify per-sensor threshold tuning.

A per-native-keyframe implementation of the exact equation (19) score was
screened in `logs/tunnel_backend_replay_paper_eq19_per_keyframe_20260821`.
It scored 429 native factors and disabled ten, but regressed causal 3-D
RMSE/P95/maximum from 3.654/8.171/12.436 m to
5.013/8.190/29.877 m and increased rollbacks from 22 to 28. The source change
was reverted. Applying a scalar modality score to a complete LiDAR factor is
still too coarse when only a subspace is degenerate.

The scheduler/backend contract was then audited. The scheduler emitted both
`reliability_weight = 1-D` and reciprocal covariance inflation, while both
backends used `weight / inflation`; this unintentionally applied
`(1-D)^2`. A single-attenuation candidate now retains the reliability weight
and uses unit inflation for enabled factors. On the same tunnel replay it
changed RMSE only from 3.654 to 3.650 m, but reduced maximum/endpoint error
from 12.436 to 11.956 m, final GNSS residual from 87.1 to 70.7 m, and final
position variance from 20.0 to 13.2 square metres. Rollbacks remained 22;
solver P95 increased from 14.93 to 20.28 ms. This is paper-consistent but is
not yet a stable navigation baseline.

Enabling conditional Schur handoff on all translation axes demonstrated that
the tunnel failure is primarily horizontal. RMSE/P95 improved to
2.551/3.839 m, but maximum/endpoint error remained 12.275 m, rollbacks rose to
32, and callback P95 rose to 6.73 s because the current conditional path
disables batched LiDAR graph linearization. A batched implementation reduced
callback P95 only to 6.33 s and regressed trajectory metrics, so it was also
reverted. Full-axis handoff remains experimental and disabled by the existing
validation defaults.

At replay rate 1.0, the healthy city bag dropped 580 of 1678 native frames,
reported 299 IMU interval timeouts, and reached solver P95 128 ms. At rate
0.5, drops fell to 20 and solver P95 to 94.8 ms, but the mid-route causal RMSE
was still 7.07 m despite a 0.193 m endpoint. This confirms that both estimator
recovery semantics and compute headroom remain unresolved; the city replay is
not accepted as a healthy regression baseline.

The replay metrics observer now stores a compact 12-field timeline and one
complete final diagnostic snapshot. The replay wrapper prints only a concise
summary instead of the full multi-megabyte JSON. Together with wall-time
accuracy observers, the MAVROS allowlist, two-second resource sampling, and
landing-triggered collector shutdown, this removes avoidable monitoring load;
the remaining RTF limit is dominated by backend linearization and callback
backlog.

## Paper Common-Backend Checkpoint - 2026-08-21

Long-scene simulation was suspended. All subsequent online checks use one
short rectangle unless a separate test plan explicitly requires otherwise.
This checkpoint changes backend semantics and initialization ownership without
tuning any sensor weight or degradation threshold.

Implemented and verified boundaries:

- the runtime OAI path now admits the stationary inertial branch `S` from FCU
  raw IMU gravity alignment and bias statistics; translation and unobservable
  yaw use a zero local gauge instead of a hidden FAST-LIO pose seed;
- a modality-neutral auxiliary worker can cold-start `S` without any LiDAR
  arrival and can continue opening candidate states while LiDAR packets remain
  present but degraded;
- after initialization, candidate states with only an IMU bridge and no
  enabled aiding factor are transactionally deferred;
- the legacy LiDAR prediction recovery floor was removed, so a failed current
  consistency gate cannot force a LiDAR factor into the graph;
- GNSS with both XY and Z consistency blocks rejected is excluded and counted
  as `gnss_all_axes_inconsistent`; one healthy block may still admit the factor
  with the failed block robustly down-weighted;
- deterministic tests prove that arbitrarily large disabled LiDAR/GNSS
  residuals do not change a healthy GNSS/flow window solution, and that a new
  degradation decision does not rewrite an older factor's stored weight;
- the raw-LiDAR calibration-motion extractor remains available, but short
  rectangle validation disables it by default because fixed extrinsics are
  authoritative and online calibration is shadow-only.

The short all-source run is
`logs/paper_backend_short_rectangle_20260821_110930`. It completed takeoff,
four rectangle legs, LAND, and FCU disarm, then stopped all collectors after a
five-second grace period. ExternalNav feedback to EKF3 was intentionally
disabled and route feedback was `fcu_local`.

Observed results were 0.0304 m causal 3-D RMSE, 0.0400 m P95, 0.0448 m maximum,
0.0202 m horizontal RMSE, 0.0228 m vertical RMSE, and 0.0236 m endpoint error.
The unified stream ran at 10.00 Hz with 0.080 s source-age P95 and no stale,
duplicate, zero, or regressing timestamps. Accepted factor counts were 845
LiDAR, 845 IMU, 419 GNSS, 183 optical-flow, and 302 visual factors according to
the strict acceptance report. The backend committed 883 states with zero
optimization errors or rollbacks. OAI reported
`S/stationary_inertial_alignment`. Solve mean/maximum were 12.42/72.81 ms;
runtime-sampled solve P95 was 45.06 ms and callback P95 was 126.51 ms.

This result is a short structural smoke test, not evidence of long-scene or
degradation robustness. RTF was only 0.320. Gazebo CPU P50/P95 was
190/220 percent, backend CPU was 37.5/48.9 percent, and other validation
processes used 384/425 percent. The calibration-motion extractor alone was
observed near one full CPU core and accepted only one calibration update from
440 motion records, motivating its short-validation default-off switch.

The paper reconstruction is not complete. Algorithm 1 branch `D` still lacks
SfM plus visual-inertial alignment for scale, gravity, velocity, and gyro bias.
Branch `M` still lacks the required short-window MAP solve for velocity and IMU
biases; a single scan-matching pose is no longer treated as a completed `M`
solution. The selector may identify these hypotheses, but runtime admission
remains deferred until their solvers return a valid result and MCC passes.
Finally, the common optimizer remains the repository's C++/Eigen-assisted LM
rather than the paper's Ceres LM. Ceres 2.0 is installed, but solver migration
must be a separate, single-variable checkpoint across every factor and prior.

Verification at this checkpoint:

- `uf_backend_fusion`: 322 pytest tests passed;
- `uf_reliability`: 81 pytest tests passed;
- `colcon test-result --verbose`: 76 aggregate result files, zero errors and
  zero failures;
- build: `uf_backend_core_cpp`, `uf_backend_fusion`, and `uf_reliability`
  completed successfully.
