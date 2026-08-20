# Dynamic integration audit of the latest relocalization line

## Audited revisions

- Latest upstream branch: `origin/exp/passive-relocalization-reinit-five-source`
- Latest upstream HEAD: `35e234dd063b16e47c1f995fce9a1758349be581`
- HEAD timestamp: `2026-08-20T18:08:46+08:00`
- Known screenshot HEAD: the same `35e234d`; there are no later commits.
- Frozen Dynamic V1: `29e580448b33cd8bd1a5815808435c2ac4a9342f`
- Frozen Dynamic tag: `dyn-loc-007-dynamic-localization-20260820`
- Common ancestor / PR #14 frozen baseline:
  `50e96f63d19e8d9292b15a684f0cc8a76f55e5bd`

The histories are genuinely divergent: upstream is five commits ahead of the
common ancestor and Dynamic V1 is seven commits ahead. `git range-diff` finds
no patch-equivalent commits. The integration must therefore preserve upstream
ownership and selectively transplant Dynamic-only files and semantics; a
blind merge is not appropriate.

## A. What upstream added after the Dynamic baseline

Upstream turns the existing relocalizer proposal into an explicit backend
epoch transaction. A successful candidate carries `map_from_lio`, source LIO
pose, recovered pose, covariance, candidate and transaction IDs. The unified
backend verifies pose/alignment consistency, timestamp freshness and monotonic
transaction identity, then waits for a newer NativeLidarFactor boundary before
committing.

At commit time it:

1. computes `epoch_correction = new_map_from_lio * old_map_from_lio^-1`;
2. resets the 15-state manifold window to the corrected pose;
3. rotates velocity by the correction by default;
4. preserves learned accelerometer and gyro biases by default;
5. optionally permits `stationary_zero` velocity and/or `stationary_imu` bias
   only after an explicit stationary IMU window and speed gate;
6. increments `state_reset_counter` and publishes `/fusion/unified/epoch`
   before the corrected state becomes externally publishable;
7. clears old-epoch visual, RGB-D, Native LiDAR, prediction, pending work,
   GNSS, flow, barometer, path, and map-eligibility state;
8. drops the in-flight old-epoch Native factor and rejects stale/future epoch
   packets.

Upstream also adds bounded active observation motions (`hold`, `yaw_scan`,
`circle`, `figure8`), checkpoint-driven request/epoch handshakes, a structural
window world, a window-opening scenario, post-reset integrity accounting, and
a scorer that separates relocalization-candidate eligibility from whole-run
deployment eligibility.

The packaged evidence recommends:

- structural-window: `hold + checkpoint 4`;
- fast motion: `figure8 + rotate velocity + preserve bias + checkpoint 4`;
- window-opening: passive hold;
- checkpoint 8: pressure/failure comparison only.

## B. Changes unrelated to Dynamic localization

The README synchronization, mission landing watchdog, route completion log,
world geometry packaging, scorer presentation, and shell orchestration are not
part of the dynamic-point algorithm. They remain upstream-owned and should be
kept unchanged unless a deterministic integration test exposes a concrete
interface bug.

The Z-axis fusion equations, GNSS-Z/barometer/RGB-D-Z policies, ExternalNav,
EKF3, one-observation-one-factor ownership, and NativeLidarFactor mathematics
were not replaced by this upstream line and are outside this integration.

## C. Direct coupling with Dynamic V1

The coupling surfaces are:

- `/fusion/unified/epoch` and the previous-posterior `reset_counter`;
- Clean Gateway causal state anchoring;
- observer voxel/free/occlusion history in the FAST-LIO `camera_init` frame;
- long-term `STATIC_CONFIRMED` map coordinates in that same LIO-local frame;
- keyframe cloud and body query cloud ownership;
- map/keyframe admission across an epoch transition;
- Native factor queue discard, epoch barrier, and post-reset integrity counts;
- active-motion scan deskew and false-dynamic behavior.

The upstream relocalizer currently builds retrieval descriptors from the raw
body-frame query scan and stores `/lio/local_map` as the dense geometric target.
Its quality gate calls these submaps static but does not itself enforce
Dynamic V1 `STATIC_CONFIRMED` ownership.

## D. Merge-conflict map

Seven files are modified on both histories:

- `src/multi_slam_uav_sim/test/test_unified_validation_result_checker.py`
- `src/ultra_fusion_nav/uf_backend_fusion/launch/online_backend.launch.py`
- `src/ultra_fusion_nav/uf_backend_fusion/uf_backend_fusion/online_backend.py`
- `tools/check_unified_validation_result.py`
- `tools/run_frozen_low_figure8_validation.sh`
- `tools/run_unified_backend_stack.sh`
- `tools/run_unified_rectangle_validation.sh`

Upstream owns relocalization reset, active motion, scoring and mission
semantics in these files. Dynamic owns runtime-contract fixes and diagnostics.
The integration policy is to retain the complete upstream versions first, then
port only demonstrably missing Dynamic runtime-contract deltas. No
`ours`/`theirs` bulk resolution is acceptable.

All `uf_dynamic_interfaces`, `uf_dynamic_observer`, long-term map, gateway,
dynamic replay and DYN reports are Dynamic-only paths and can be transplanted
without text conflicts. The FAST-LIO previous-posterior patch is also
Dynamic-only but must remain pinned to the audited external source revision.

## E. Algorithm-semantic conflict

The main semantic conflict is coordinate ownership after reinitialization.
Upstream corrects `map_from_lio`, resets the unified backend epoch, and
deliberately invalidates backend old-epoch buffers. It does **not** reset or
re-anchor FAST-LIO's `camera_init` frame. `/lio/local_map`, the previous
FAST-LIO posterior, and the body scan therefore remain mutually consistent in
LIO-local coordinates. The relocalizer applies the newest `map_from_lio` when
it projects that local map into unified `map` coordinates.

The published `FusionEpoch` contains identifiers but not the correction
transform. That prevents a cross-process consumer from safely transforming a
unified-map history, but no transform is needed for Dynamic history that is
explicitly owned by `camera_init`. Clearing that history for every accepted
backend relocalization would discard valid free/static evidence and create an
avoidable fail-open interval.

There are consequently two distinct events:

- an applied backend `FusionEpoch` is recorded diagnostically and retains all
  LIO-local Dynamic history;
- an increase in `PreviousFastLioState.reset_counter` means the FAST-LIO local
  epoch itself changed. That event invalidates the causal anchor and local
  voxel coordinates: pending scans are passed through raw, short-term history
  is cleared, the long-term map is quarantined by clearing its transient-local
  output, and exactly six healthy causal scans rebuild evidence before clean
  filtering resumes on the following scan.

Failed, unapplied, duplicate, stale backend epochs and stale LIO reset counters
cannot mutate Dynamic state. This split avoids both stale-frame reuse and
unnecessary loss of valid local evidence.

## F. Screening evidence versus deployment evidence

The upstream score table is careful but mostly screening evidence. Many rows
have one run. Fast figure8/rotate/checkpoint4 and structural hold/checkpoint4
have two runs and are still labelled screening. Structural active rotate is
unstable (one of two eligible); checkpoint8 is poor; the window-opening passive
result is one run and includes cumulative rather than post-reset integrity
fallback. None of these rows alone establishes a production policy.

The scorer correctly distinguishes post-reset integrity deltas when present,
falls back to cumulative counts for older logs, and requires runtime completion
plus LAND/disarm for deployment eligibility. This integration will preserve
that definition and will not introduce parallel integrity counters.

## G. Recommended policy for this integration

1. Keep upstream relocalization, mission, checkpoint, epoch, scoring and
   structural-scene implementations as authoritative.
2. Keep `rotate + preserve` as the general reset policy. Use
   `stationary_zero + preserve` only for a verified passive hold; never infer
   stationary state from active motion.
3. Use structural `hold/checkpoint4`, fast `figure8/rotate/checkpoint4`, and
   window-opening passive hold as primary comparisons. Keep fast checkpoint8
   only as a negative pressure control.
4. Add an explicit Dynamic epoch guard: retain history for backend-only map
   epoch changes; clear/quarantine and fail open through bounded reseeding only
   for a genuine FAST-LIO local reset-counter transition.
5. Preserve Raw input and UNKNOWN points.
6. Compare relocalization maps as four ownership strategies: Raw/Raw,
   Clean/Clean, Refined/Refined, and hybrid Refined candidate retrieval plus
   Clean high-confidence final registration. Prefer the hybrid only if it
   retains zero false matches while recovering Clean-level registration
   accuracy.
7. Treat complete persistent occlusion as physically unobservable until a
   measured ray reobserves the region.
