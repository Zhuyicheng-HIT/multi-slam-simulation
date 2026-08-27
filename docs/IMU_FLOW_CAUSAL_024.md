# IMU-FLOW-CAUSAL-024

## Scope

This audit reuses the HXY-DIAG-002 frozen bag and the `rgbd_direct` dual
LiDAR/GNSS-degradation setup from VISUAL-PRIOR-023. FAST-LIO IMU input,
integrity limits, HXY, Flow thresholds, GNSS/IMU weights, Dynamic, and formal
marginalization mathematics are unchanged. Truth is used only by the existing
offline scorer.

The bag SHA256 remains
`180d4f0e8985ae7253dfa8edac242f0dd9f814c2b4d1c7b93c0088351c0375a9`.

## Active-window timeline

The authoritative no-extra-diagnostic-hot-path replay is
`logs/visual_ab_022/direct_dual/backend_cycle_trace.jsonl`.

A fresh baseline launch was also attempted under the same nominal parameters.
As already observed in VISUAL-PRIOR-023, asynchronous visual association is
timing-sensitive: that launch admitted 12 rather than 11 visual factors, did
not reproduce transaction 313, and later drifted to 25.37 m XY RMSE. It is not
a valid replacement baseline and is retained only as timing-sensitivity
evidence. The comparison below therefore uses the frozen authoritative A
trace and the new factor-only B traces; truncated post-lock accuracy scores are
not interpreted as estimator improvement.

| Event | Transaction | Time | Evidence |
| --- | ---: | ---: | --- |
| Last `rgbd_direct` factor added | 297 | 60.324 s | one visual factor added |
| Last active LiDAR factor ages out | 306 | 61.215 s | active = prior 1 + IMU 7 |
| Last successful commit | 313 | 61.908 s | active = prior 1 + IMU 7 |
| First rollback attempt | 313 | 62.007 s | same active factors, one fresh IMU factor attempted |

The window has eight states. Each successful new transaction marginalizes the
oldest state/factors, so the last current visual factor leaves the active
window and its information survives only in the Schur prior. GNSS is disabled
by the dual-degradation replay mask. LiDAR is no longer admitted and its last
active factor has aged out by transaction 306. Flow never forms a factor (see
below). Therefore the only factor connecting each new state to the retained
history is IMU preintegration, while the retained absolute-like historical
anchor is the marginal prior.

At 62.007 s the solver proposes a correction dominated by negative Y:

```
translation approximately [+0.100, -1.283, -0.046] m
velocity    approximately [+0.120, -1.289, +0.005] m/s
rotation norm approximately 0.065 rad
```

The archived non-vector trace reports translation and velocity norms of
1.276 m and 1.282 m/s. The vector was captured by the targeted prior
diagnostic replay, whose small timing overhead slightly changed the norm but
not the transaction, sign, direction, or failure mechanism.

After rollback the transaction snapshot is restored. Sensor time advances but
transaction 313 does not, producing 159 consecutive
`excessive_translation_correction` rollbacks through 77.913 s. This is the
previously identified state/prediction self-lock.

## Optical-flow exit layer

Flow is present at the backend input in this frozen bag, but it never enters
the solver. Representative completed runs record 937 received messages,
hundreds of factor attempts, zero solver factors, and
`last_flow_reason=scheduler_disabled`.

The exact exit order in `_flow_factor` is:

1. select timestamped Flow records;
2. query the recorded scheduler decision;
3. return with `scheduler_disabled` when `factor_enabled=false`;
4. only after that return point would the backend compute displacement,
   ground-distance/quality validity, MID360 gyro compensation, rotation gate,
   and the solver factor.

Consequently the 62 s interval has no Flow constraint because the recorded
scheduler decision rejects it upstream of all repaired observation math. The
frozen bag was created on 2026-08-21; the simulation Flow input-contract fix
(`1ff485c`, 2026-08-26) changed the live world/generator route and cannot
retroactively change the recorded Flow/scheduler messages. FLOW-CONTRACT-021's
post-fix moving/turning simulation is separate evidence (708/873 solver
admissions), not evidence that this old frozen bag contains repaired Flow.

## IMU factor ablation

A diagnostic-only parameter skips only `add_imu_preintegrated` while retaining
the IMU subscription, ordered samples, preintegration, initialization,
propagation, and FAST-LIO IMU path. It defaults off. A simulation-time cutoff
allows the same history to be retained up to 60.0 s.

| Replay | First reject | First Y correction | First Y velocity correction | Progress |
| --- | --- | ---: | ---: | --- |
| Baseline | tx313, 62.007 s, excessive correction | about -1.283 m | about -1.289 m/s | 313 commits, then frozen; 159 rollbacks |
| IMU solver factor off from start | tx1, 30.822 s, latest information ill-conditioned | 0 m | 0 m/s | 33 intermittent commits; 357 rejected attempts; no usable visual trajectory |
| IMU solver factor off from 60.0 s | tx293, 60.027 s, latest information ill-conditioned | 0 m | 0 m/s | 294 commits, then frozen; 166 rejected attempts |

The full-off run still receives 7,415 IMU samples and FAST-LIO is unchanged,
but adds zero backend IMU factors. The cutoff run adds 292 IMU factors before
60 s and keeps seven historical IMU factors in the window. At 60.027 s the new
15D state has no IMU edge; pose-only LiDAR/visual factors do not constrain its
velocity and two bias blocks. The integrity check therefore correctly reports
rank-deficient latest-state information before any translation update.

This is an important negative result: removing the backend IMU factor does not
produce a meaningful alternative trajectory against which to score Y drift.
It removes the only factor that makes the newest full navigation state
observable. The ablation rules out IMU-off as a repair, but cannot by itself
assign a scalar percentage of the baseline rollback to IMU propagation.

## Causal interpretation

The evidence supports a combined mechanism, with distinct roles:

1. **Missing current weak-direction constraint is structural.** After 61.215 s
   there is no current LiDAR, GNSS, Flow, or visual factor. Fresh IMU is the
   only bridge to the new state and cannot supply absolute horizontal
   observability.
2. **IMU bias/propagation mismatch supplies the immediate proposal.** The
   baseline solve changes accel bias by 0.0235 m/s2 and gyro bias by 0.0113
   rad/s while proposing nearly equal negative-Y position and velocity
   corrections. That coupled signature is consistent with fresh
   preintegration pulling the propagated newest state relative to the retained
   history. It is not evidence of an IMU unit/frame failure; startup, sample
   continuity, and normal preintegration remain valid.
3. **Historical visual linearization is part of the retained anchor.** Near
   62 s, source attribution records 11 historical `rgbd_direct` factors in the
   prior. The prior is finite and positive-rank (full condition about 365,
   position trace about 48.3), not globally indefinite. The prior-only visual
   exclusion from VISUAL-PRIOR-023 delayed the authoritative failure but caused
   severe later drift, proving the history is useful and also required for the
   early conflict. This supports a stale/inconsistent historical
   linearization in the weak direction, not blanket removal or proof that the
   prior is simply too strong.
4. **Integrity is the final detector, not the source.** It sees an already
   generated correction above 1.0 m, rejects it, and restores the snapshot.
   Reusing that snapshot with advancing measurements then creates the lock.

Thus all three proposed contributors participate: IMU propagation/bias error
creates relative drift, missing current Flow/visual leaves it uncorrected, and
the historical visual prior supplies a different old linearization anchor.
The strongest root cause is the loss of a current horizontal observation at
the prior-to-new-state boundary; the large correction is the resulting
prior/IMU consistency failure. Available evidence does not justify calling the
IMU model alone or the prior strength alone the root cause.

## Minimum next step

Do not disable IMU and do not relax integrity. Create a new post-FLOW-CONTRACT
moving/turning capture with the same dual-degradation interval and verify Flow
factors remain active across 61-63 s. Then repeat the prior source residual
diagnostics. If rollback remains with a current Flow factor, implement the
narrow visual-prior consistency policy proposed in VISUAL-PRIOR-023:
relinearize or attenuate only the inconsistent historical visual weak
component, preserving strong metric depth directions and all current factors.

No production estimator behavior was changed by this task.
