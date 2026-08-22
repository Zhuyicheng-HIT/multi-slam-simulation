# Tunnel directional information handoff - 2026-08-22

## Scope

This experiment uses the unchanged tunnel bag from
`logs/large_scene_tunnel_static_heartbeat_grace_20260820/replay_bag`.  Gazebo
truth never enters the estimator.  Accuracy is evaluated afterward against
the valid online `apm_iris` truth samples retained by the original run; the
bag's static `/sim/mid360/ground_truth_odom` topic remains excluded.

All replays use 0.5 playback rate, a 10-state manifold window, up to eight
nonlinear iterations, the C++ math core, dynamic FRS, RGB-D direct factors,
and latest-only native LiDAR processing.  RangeFacet, barometer fallback,
GNSS Z reanchor, and online calibration application remain disabled.

## Change

The native LiDAR conditional translation normal is eigendecomposed after
rotation has been Schur-eliminated.  A symmetric information transform
attenuates only weak eigendirections for which a fresh independent factor has
information.  The LiDAR conditional optimum and all rotation/coupling blocks
are preserved.

GNSS factors can carry a full 3-by-3 information matrix.  When raw GNSS health
and temporal gates pass but the common-state NIS gate rejects all XYZ blocks,
an optional recovery path projects GNSS information into the LiDAR weak
subspace only.  No information is added in the LiDAR strong complement.
Ordinary admitted GNSS information is also retained as a full matrix for the
same-cycle LiDAR handoff calculation.

The feature is controlled by `subspace_information_handoff_enabled` and is
disabled in the default online configuration.

## Deterministic A/B results

The table uses one common post-processing implementation: replay estimates
are interpolated at the historical truth timestamps and a translation-only
alignment is frozen from the first 10 seconds of overlap.

| Run | 3-D RMSE | 3-D P95 | 3-D max | XY RMSE | Z RMSE | Endpoint |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A, existing XYZ/Z handoff | 31.840 m | 58.628 m | 488.713 m | 30.814 m | 8.020 m | 488.713 m |
| B, rotated LiDAR handoff only | 156.796 m | 289.774 m | 1826.494 m | 148.540 m | 50.210 m | 1826.494 m |
| B2, strong-subspace GNSS recovery gate | 163.971 m | 302.431 m | 1911.825 m | 154.422 m | 55.138 m | 1911.825 m |
| B3, full projected alternative information | 1.010 m | 2.313 m | 3.568 m | 1.010 m | 0.018 m | 1.311 m |

Evidence directories:

- A: `logs/tunnel_current_backend_replay_20260822_102921`
- B: `logs/tunnel_subspace_handoff_replay_20260822_112252`
- B2: `logs/tunnel_subspace_gnss_recovery_replay_20260822_113443`
- B3: `logs/tunnel_subspace_projected_gnss_replay_20260822_114205`

B and B2 demonstrate that LiDAR-only attenuation is insufficient once GNSS
has already been rejected by common-state innovation.  B3 prevents that
failure earlier: GNSS factors increase from 214 in A and 180 in B/B2 to 289,
while whole-factor GNSS NIS rejection falls from 67/105/105 to zero.  The
explicit post-rejection GNSS subspace recovery count is zero in B3, so the
improvement comes from preventive same-cycle handoff, not repeated recovery
after divergence.

## Runtime and factor evidence

| Item | A | B3 |
| --- | ---: | ---: |
| Committed states | 577 | 577 |
| Transaction rollbacks | 0 | 0 |
| Native queue discarded | 0 | 0 |
| GNSS factors | 214 | 289 |
| GNSS whole-factor NIS rejected | 67 | 0 |
| Optical-flow factors | 0 | 0 |
| RGB-D direct factors | 17 | 6 |
| Solver P95 | 16.998 ms | 18.877 ms |
| Callback P95 | 42.603 ms | 38.821 ms |
| Maximum callback | 53.228 ms | 43.646 ms |
| LiDAR prediction-gate rejections | 77 | 115 |
| Final position variance | 168691.8 m2 | 170082.5 m2 |

Build and test evidence after B3: both `uf_backend_core_cpp` and
`uf_backend_fusion` build successfully; the backend suite runs 317 tests with
no failures; `colcon test-result --verbose` reports 76 records, zero errors,
and zero failures.

## Decision

B3 is retained as a default-off experimental candidate because it reduces the
fixed-tunnel 3-D RMSE by about 96.8 percent without a runtime-tail regression.
It is not a frozen baseline.  XY RMSE remains about one metre, the LiDAR
prediction gate rejects 115 frames, no optical-flow factor is admitted, RGB-D
admission falls to six factors, and the reported marginal covariance remains
unphysically large.  The next fixed-data iteration should address prediction
gating and covariance consistency without changing the directional handoff
or adding another sensor-policy variable in the same A/B.
