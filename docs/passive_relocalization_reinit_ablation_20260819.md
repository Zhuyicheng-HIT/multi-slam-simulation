# Passive relocalization reinitialization ablation (2026-08-19)

## Scope and baseline

- Branch: `exp/passive-relocalization-reinit-five-source`
- Frozen source: `checkpoint/low-altitude-externalnav-20260819`
- Mainline checked before every policy group: `origin/feat/core-algorithm-cleanup-20260817` at `7441e4f`
- Mainline algorithm delta from the frozen source: none (`README.md` only)
- Flight: frozen low-altitude figure-eight, passive checkpoint trigger only
- Sensors: LiDAR, IMU, GNSS, optical flow, and RGB-D visual frontend
- EGO active relocalization: disabled
- Relocalization acceptance thresholds: unchanged

The backend always starts a new fusion epoch after an accepted alignment. It
clears factor and sensor-history buffers, publishes the new epoch before the
corrected state, and rejects old-epoch factors. This experiment changes only
how velocity and IMU bias are initialized in that new epoch.

## Policies

| Profile | Requested velocity | Requested bias | Guard |
| --- | --- | --- | --- |
| `baseline` | rotate into the corrected map frame | preserve | none |
| `stationary_velocity` | zero | preserve | stationary IMU window and pre-reset speed <= 0.35 m/s |
| `stationary_velocity_bias` | zero | estimate from stationary IMU | same guard |

If the stationary guard fails, both requested stationary operations fall back
to `rotate/preserve`. This prevents a short or moving IMU window from being
treated as a trustworthy Ultra-Fusion-style reinitialization.

## Results

All accuracy values use the frozen initial-alignment, causal metric. The
acceptance threshold is 0.20 m.

| Profile | Actual stationary init | 3D RMSE (m) | P95 (m) | Max (m) | Endpoint (m) | Accuracy gate |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `baseline` | not requested | 0.2388 | 0.3547 | 0.8175 | 0.2702 | fail |
| `stationary_velocity` | accepted, velocity zeroed | 0.0961 | 0.1236 | 0.1641 | 0.1245 | pass |
| `stationary_velocity_bias` | rejected, fallback used | 0.0920 | 0.1218 | 0.1288 | 0.1166 | pass |

The bias profile was rejected because the IMU angular-rate variation exceeded
the existing startup limit. Its good trajectory is therefore a fallback run,
not evidence that bias reseeding helps. Run-to-run candidate selection also
varied: baseline selected keyframe 12, while the two stationary-requested runs
selected keyframe 7. A larger repeated sample is required before attributing
all accuracy change to the velocity policy.

The accepted transaction wall-clock recovery times were 3.72 s, 7.81 s, and
7.05 s respectively. These include search time and are not a pure backend reset
latency comparison.

## Negative sample

Triggering the baseline profile at checkpoint 4 was safely rejected before any
state reset. The best and alternative verified candidates had a score margin
of 0.0088, below the unchanged 0.05 minimum, and represented separated poses.
This supports using checkpoint 8 for the controlled reinitialization ablation;
it does not justify weakening the ambiguity gate.

## Evidence boundary

- Verified: all three checkpoint-8 runs committed one fusion epoch, resumed the
  route, completed the figure-eight, and landed/disarmed.
- Verified: `unified_accuracy.json` and `unified_runtime_metrics.json` completed
  for all three runs; runtime termination was `duration_complete`.
- Partial: `analyze_slam_drift.py` returned failure in these low-route runs, so
  the wrapper exited nonzero after metrics were collected. This monitor uses a
  different FAST-LIO diagnostic and is not the unified causal accuracy gate.
- Not verified: repeated-run confidence intervals, accepted IMU bias reseeding,
  hardware behavior, EGO interaction, or closed-loop fault recovery without a
  scripted checkpoint hold.

## Decision

Keep `rotate/preserve` as the default frozen behavior. Continue experiments
with `stationary_zero/preserve` as the leading passive recovery candidate, but
do not merge it into the main flight path until repeated checkpoint-8 runs and
at least one additional non-repetitive location reproduce the improvement.
Keep `stationary_imu` experimental until a valid stationary window is obtained;
do not relax the stationary IMU gate merely to force acceptance.

Run a profile with:

```bash
VALIDATION_ROS_DOMAIN_ID=64 \
RELOCALIZATION_REINIT_PROFILE=stationary_velocity \
VALIDATION_RELOCALIZATION_CHECKPOINTS=8 \
LOG_DIR="$PWD/logs/passive_relocalization_stationary_velocity_checkpoint8" \
bash tools/run_passive_relocalization_reinit_validation.sh
```
