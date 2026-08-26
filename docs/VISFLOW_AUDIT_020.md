# VISFLOW-AUDIT-020: visual and optical-flow constraint audit

## Scope

- Branch: `feat/lidar-horizontal-degeneracy-v1`
- Audited HEAD: `27c79b9876db741ab17e28ff831bf5d1166a2e47`
- No estimator parameter, HXY, GNSS/IMU weight, or Dynamic change was made.
- The audit reuses existing simulation and replay evidence. It does not treat a
  fixed-zero sensor ablation as an observation-admission experiment.

The backend counters use three different denominators. `received` is input
traffic, `attempts` is a factor candidate associated with a state, and
`factors` is an admitted solver factor. Adoption below is therefore reported
as `factors / attempts`, with `factors / received` added when it matters.

## Evidence used

1. Fault-free 60 s static simulation after Dynamic integration:
   `logs/static_after_dynamic_009_clean2/runtime_evidence.json`.
2. Frozen long-tunnel replay, geometrically degenerate LiDAR with healthy GNSS:
   `/home/ld666/projects/hxy-diag-002/prior_007_baseline/backend.log`.
3. Same frozen input with PR17-style zero LiDAR solver admission, used only as
   a causal contrast for optical-flow suppression.
4. Historical Robustness V3 reports for existence of GNSS and dual-degradation
   profiles. Their ignored machine reports are no longer present, so they do
   not provide auditable visual/flow factor counts.
5. M2DGR online replay with `rgbd_direct`:
   `/home/ld666/ultrafusion-datasets/reports/m2dgr_plus_anomaly_official_extrinsics_flu_20260820/unified/online_backend.log`.
   This is solver-path evidence, not simulation evidence.

## Four-condition accounting

| Condition | Comparable current artifact | Optical flow | `paper_reprojection` | Main rejection |
|---|---|---:|---:|---|
| Fault-free sensor inputs (60 s static simulation) | Yes | 0 / 597 attempts (0%) | 23 / 33 attempts (69.70%); 23 / 285 received (8.07%) | Flow evidence invalid; visual pending association expiry |
| LiDAR degraded, GNSS healthy (frozen tunnel) | Yes | 0 / 671 attempts (0%) | 2 / 57 attempts (3.51%) | Flow scheduler disabled; visual PnP observability |
| GNSS degraded, LiDAR non-degenerate | No comparable retained factor-accounting run | Not measured | Not measured | Existing GNSS ablation fixed visual/flow weights to zero and is invalid for adoption analysis |
| LiDAR + GNSS degraded | No comparable retained factor-accounting run | Not measured | Not measured | Historical campaign proves the profile ran, but its per-factor machine evidence is absent |

This matrix is intentionally incomplete rather than substituting incompatible
runs. The existing four-profile robustness campaign is sufficient to show
trajectory continuity, but not to answer current visual/flow admission rates.
The retained GNSS-only and LiDAR-only HXY replays used fixed reliability with
`fixed_optical_flow_weight=0` and `fixed_vision_weight=0`; their zero factors
are configuration outcomes, not rejection evidence.

## Optical flow

### Where adoption is lost

In the 60 s simulation, all 909 recorded optical-flow score samples were
invalid, with reliability weight zero. Every sample carried the same evidence
failures:

- `increment_prediction_unavailable_eq22_adapted`
- `low_quality_extension`
- `invalid_ground_distance_extension`
- `incomplete_paper_evidence`
- `flow_rotation_gate_marks_observation_unavailable`
- `gyro_compensation_unavailable`

Consequently the backend attempted 597 state-aligned flow candidates but
formed no factor. The backend counters do **not** attribute these losses to its
quality, speed, rotation, or clock-mismatch rejection counters; the scheduler
has already disabled the modality before those gates can admit a factor.

The score model permits a prediction-free fallback using flow quality and
ground distance. It cannot activate here because the same observations also
have low quality, invalid distance, and unavailable gyro compensation. The
first fault is therefore the simulated optical-flow observation/compensation
contract, not a backend factor threshold.

### Does degenerate LiDAR suppress flow?

There is no direct scheduler coupling in which LiDAR degradation lowers the
optical-flow modality weight. A LiDAR-derived motion prediction can affect the
Eq. 22-adapted increment residual indirectly, so an unhealthy or unavailable
prediction is a possible secondary dependency. The implementation's
quality-and-distance fallback exists specifically to avoid making that
dependency mandatory.

Current evidence does not support reverse suppression as the primary cause:

- flow is already 0 / 597 in the fault-free static input condition;
- it remains 0 / 671 in the degenerate tunnel;
- PR17-style replay with no LiDAR solver factors still remains 0 / 671 and is
  scheduler-disabled at essentially the same rate;
- the actual flow samples fail quality, distance, and gyro-compensation
  evidence independently of LiDAR solver admission.

Therefore disabling or attenuating LiDAR will not make the present flow stream
usable. A later moving A/B test should still compare the same healthy raw flow
with and without LiDAR prediction, but only after the raw flow evidence is
valid.

## Visual factors

### `paper_reprojection`

The mode can stably enter the solver. In the 60 s simulation it formed 23 of 33
associated candidates, and all 23 were solver-accepted with zero solver
rejection. The low `23 / 285 received` end-to-end rate is mostly association
latency: 192 pending candidates expired, 60 were pre-bootstrap, and 8 associated
candidates failed state-consistency reprojection checks. Frontend quality
rejected only one candidate (`pnp_invalid`).

In the long tunnel, the end-to-end association is no longer the dominant
problem: all 57 received candidates reached factor attempts, but only 2 formed
factors. The accounted rejection chain is:

| Tunnel outcome | Count | Share of 57 attempts |
|---|---:|---:|
| PnP observability rejected | 40 | 70.18% |
| Insufficient geometric tracks | 8 | 14.04% |
| State consistency rejected | 5 | 8.77% |
| Other non-factor attempts | 2 | 3.51% |
| Solver factor formed | 2 | 3.51% |

The dominant tunnel failure is thus feature geometry/PnP observability, not
GNSS admission and not evidence that the solver rejects valid visual factors.
The backend separately reports three initialization batches; that lifecycle
counter overlaps candidate processing and is not used as an exclusive attempt
outcome.

### `rgbd_direct`

The factor implementation and solver path are functional, but the current
simulation evidence does not establish its four-condition behavior. The
retained M2DGR online replay formed 88 of 107 attempted direct factors
(82.24%, or 88 of 211 received). Its final factor was
`accepted_rgbd_direct`, with 32 tracks and bounded depth/photometric prefit;
the photometric term was downweighted rather than rejected.

The current static simulation and frozen HXY replay ran
`paper_reprojection`; their summaries report `rgbd_direct_reason=disabled`.
Therefore the defensible conclusion is:

- `rgbd_direct` can stably enter this backend solver;
- it has not yet been validated in the existing simulation under normal,
  GNSS-degraded, LiDAR-degraded, or dual-degraded conditions.

## Conclusions and next priority

1. Optical-flow adoption is low because the simulated source fails its basic
   measurement contract: valid ground distance, usable quality, and
   time-aligned gyro compensation are missing. Parameter tuning at the backend
   cannot repair this and would hide the first failure.
2. There is no evidence that degenerate LiDAR solver admission is what disables
   flow. LiDAR motion prediction is an indirect dependency, but the raw flow
   remains independently unusable.
3. `paper_reprojection` is solver-capable. Its normal static bottleneck is
   pending-state association; its tunnel bottleneck is PnP geometry.
4. `rgbd_direct` is solver-capable outside simulation and is a promising bypass
   for tunnel PnP geometry, but it lacks a current simulation admission test.
5. The formal four-condition adoption matrix is not yet complete. Existing
   fixed-zero ablations must not be used to fill its missing cells.

The next item should be the optical-flow input contract, in this order:

1. trace the simulated range/ground-distance field into
   `/sensors/optical_flow/rad`;
2. verify quality generation and units;
3. verify MID360 IMU gyro timestamp/frame association used for compensation;
4. confirm prediction-free fallback becomes valid with otherwise healthy raw
   flow;
5. then run one frozen moving capture through four controlled profiles:
   nominal, GNSS outage, LiDAR correspondence degradation, and both together.

That replay must retain dynamic reliability and record, per modality and per
profile, `received`, score-valid, scheduler-enabled, state-associated attempts,
formed factors, solver-accepted factors, and rejection reasons. After flow is
observable, the second priority is to enable `rgbd_direct` on the same capture
and compare it with `paper_reprojection` without changing either mode's
thresholds.
