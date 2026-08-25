# HXY-INTERACTION-006: pre-transaction-331 horizontal drift

## Scope and replay contract

Frozen input: `/home/ld666/projects/hxy-diag-002/frozen_bag` (metadata SHA256 `d842a6d3e19159123644efb8e9ac0d80e46ac2b02360a1ed44b812e84222372b`, db3.zstd SHA256 `1fdf2c5616670dc9fca7d6ba830ac5b2a4eec07d9cd54deb6f3d6aaf965ebd56`). Runs used commit `f82a0d96ae5e1d33b74f0335d10c3fecc7f272c3`, one numeric thread, queue/QoS depth 1024, `latest-only=false`, rate 0.4, thresholds 0.15/0.25 and weak scale 0.001. Truth was used only offline.

The trace records admitted GNSS solver information, cached rotation-conditioned Schur translation information for every active LiDAR factor, weak-direction projections, and ratios. It does not add or remove factors or alter the normal equation.

## Isolation results

| replay | XY RMSE | 3D RMSE | endpoint norm | observation |
|---|---:|---:|---:|---|
| full, original | 1.538 m | 1.538 m | 6.99 m | stopped after rollback chain |
| LiDAR + IMU | 55.376 m | 55.571 m | 141.45 m | no GNSS factors |
| GNSS + IMU | 0.787 m | 0.788 m | 2.09 m | absolute constraint stabilizes |
| full, rollback off | 1.158 m | 1.158 m | 2.04 m | first drift remains |
| full, diagnostic | 0.799 m | 0.799 m | 2.05 m | no queue loss |
| weak-mode cap | 0.790 m | 0.790 m | 2.05 m | residual remains |

Exact values vary slightly with ROS executor interleaving, but the first 300 transactions are stable across full and rollback-off runs.

## First causal divergence

The first persistent 5 cm horizontal error is transaction 209 at `t=51.513 s`; the first persistent 0.2 m error is transaction 283 at `t=58.905 s`; the first persistent 1 m error is transaction 300 at `t=60.621 s`. GNSS was admitted whenever a sample was available, with no GNSS NIS, integrity, stale, jump, or scheduler rejection before this drift.

The LiDAR weak direction is nearly world Y (`|direction_y| > 0.999`). Before the repair, active-window LiDAR weak information was median 5.66 times GNSS weak information (P95 9.58; transaction 200: 65.73 versus 10.45). Valid GNSS corrections were therefore admitted but repeatedly out-informed by accumulated weak-direction LiDAR factors. LiDAR+IMU reproduces the same growth; GNSS+IMU suppresses it.

Transaction 331 (`t=63.723 s`, scan 461) is later: it is the first prediction-gate rejection, not the origin. The first original rollback is transaction 340 (`t=64.713 s`) with an excessive translation correction. Rollback-off preserves the tx209/283/300 drift and only prevents later amplification/truncation.

## Minimal repair and regression

The repair adds a one-sided information cap inside the existing active weak-subspace episode. It uses cached solver Schur blocks and GNSS information actually admitted in that transaction. Weak-mode LiDAR information is capped at a 1:1 ratio; GNSS weights/covariance, factor count, strong modes, Z, state machine, relocalization, and marginal-prior mathematics are unchanged. The cap cannot undo information already marginalized.

The regression suite has 174 passing tests, including a rotated weak-mode test proving strong-mode scale remains exactly one. The prototype reduces active-window weak-mode ratio to about 0.10 median, but whole-run RMSE remains statistically the same as GNSS+IMU. The first mechanism is confirmed; remaining error is outside a current-window-only intervention, chiefly prior history and GNSS/IMU replay residuals.

## Decision

Root cause confirmed: valid GNSS was not rejected or rolled back at first drift; it was persistently out-informed in the weak LiDAR subspace. Rollback is a secondary amplifier after transaction 331. Keep the cap as a diagnostic repair, but do not promote it as a complete accuracy solution. Next isolate already-marginalized history and absolute-factor timing before changing marginalization mathematics.
