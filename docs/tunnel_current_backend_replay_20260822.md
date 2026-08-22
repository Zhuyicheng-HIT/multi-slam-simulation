# Current-backend fixed tunnel replay - 2026-08-22

## Scope

This run evaluates commit `c3a7207` against the unchanged pre-correction bag
from `logs/large_scene_tunnel_static_heartbeat_grace_20260820/replay_bag`.
It does not regenerate Gazebo sensor data and does not change estimator
weights.  The current reliability monitor and scheduler were regenerated from
the recorded raw evidence instead of replaying the historical scheduler
decisions.

The replay used the frozen five-source boundary: a 10-state window, at most
eight nonlinear iterations, RGB-D direct factors, dynamic FRS, Z-only axis
handoff, and disabled RangeFacet, barometer fallback, GNSS Z reanchor, online
time-offset application, and online extrinsic application.  Playback rate was
0.5 to prevent compute-induced latest-only queue loss.

Evidence: `logs/tunnel_current_backend_replay_20260822_102921`

## Verification

- Relevant package build: pass (7 packages).
- `uf_backend_fusion`: 309 tests pass.
- `uf_reliability`: 81 tests pass.
- `uf_sensor_pipeline`: 37 tests pass.
- Combined colcon result: 76 test records, zero failures or errors.
- Bag playback status: 0.
- Recorded native LiDAR factors: 575; received: 575.
- Native latest-only queue discarded/superseded: 0/0.

## Accuracy

The bag's `/sim/mid360/ground_truth_odom` is the previously withdrawn static
sensor pose and is not used for the result below.  Current replay estimates
were interpolated at the timestamps of the valid online `apm_iris` truth
samples retained in the original run.  A translation-only alignment was
frozen from the first 10 seconds.

| Metric | Historical online failure | Current fixed-data replay |
| --- | ---: | ---: |
| Causal 3-D RMSE | 31.654 m | 31.773 m |
| Causal 3-D P95 | 83.279 m | 58.608 m |
| Causal 3-D maximum | 157.655 m | 488.714 m |
| Causal XY RMSE | 31.642 m | 30.748 m |
| Causal Z RMSE | 0.857 m | 8.003 m |
| Endpoint error | 157.655 m | 488.714 m |

There is a 5.301 s gap in the historical online truth samples while the old
backend was stalled.  Over the continuous association segment ending at
81.807 s, the current replay has 3-D RMSE/P95/maximum of
22.828/59.125/72.168 m.  The final valid truth sample at 88.475 s is retained
for the table because the current estimate is available at the same source
time; its error is 488.714 m.  Reporting both scopes prevents the truth gap
from being mistaken for either continuous coverage or a reason to discard the
valid endpoint failure.

## Factor and runtime evidence

| Item | Current replay |
| --- | ---: |
| States committed | 577 |
| Transaction rollbacks | 0 |
| Rejected and recovered aiding transactions | 17 / 17 |
| Native LiDAR relinearized | 422 |
| Final native translation rank / condition | 2 / infinite |
| IMU received / factors | 6031 / 576 |
| GNSS received / factors | 308 / 214 |
| GNSS XY NIS rejected | 152 |
| GNSS XY robust-downweighted | 85 |
| Optical flow received / factors | 717 / 0 |
| RGB-D direct received / factors | 45 / 17 |
| Solver P50 / P95 / max | 9.013 / 16.998 / 32.233 ms |
| Callback P50 / P95 / max | 30.080 / 42.602 / 53.228 ms |
| Final / maximum position variance | 168691.8 / 774322.0 m2 |

## Decision

The current backend is computationally cleaner on this fixed replay: it
consumes all native packets, commits more states, avoids rollback, and keeps
solver and callback tails bounded.  It does not satisfy the structural-
degeneracy navigation objective.  Longitudinal error still grows first, the
common state later rejects or downweights healthy GNSS, no optical-flow factor
is admitted, and uncertainty explodes.  The lower P95 must therefore not be
promoted as a stable baseline because maximum, endpoint, and Z errors regress
substantially.

The next correction must preserve independently healthy absolute-position
evidence before the LiDAR-corrupted state can make cross-modal innovation
authoritative.  The same bag should remain the first deterministic regression
gate after that change.
