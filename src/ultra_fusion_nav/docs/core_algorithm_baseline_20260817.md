# Core Algorithm Baseline - 2026-08-17

## Scope

This document freezes the current algorithm boundary before further Z-axis
work. It separates the usable core from experiments that must not be enabled by
default.

The pre-cleanup working tree is archived at commit `af1bd50` on branch
`archive/pre-core-cleanup-20260817`. The cleanup branch is
`feat/core-algorithm-cleanup-20260817`.

## Non-negotiable factor ownership rule

One physical sensor observation may enter the sliding window exactly once.
The observation may be split by axis or scaled by reliability, but it must not
be represented by multiple factors in the same optimization transaction.

- IMU: one preintegrated factor for one state interval. Re-integration replaces
  the interval linearization; it does not append a second IMU factor.
- MID360: one scan contributes either raw point-to-plane correspondences or one
  condensed normal/Hessian representation, never both.
- GNSS/BDS: one selected fix contributes one 3-D position factor. XY and Z may
  receive different information scales, but they remain one factor record.
- MTF-01P: one integration window contributes one horizontal 2-D displacement
  factor. Range is used for optical-flow scale, not as a second Z factor.
- D435i: one feature batch contributes either the paper-style reprojection
  factor or the experimental RGB-D depth geometry factor, never both.
- Barometer: pressure is a separate physical observation and, when explicitly
  enabled, contributes one local relative-Z factor per selected interval.

Dynamic Z gauge has been removed. GNSS-Z is no longer routed to a second frame
or used to move a `fusion_map -> camera_init` transform outside the window.

## Current usable baseline

The most trustworthy historical frozen-bag reference is
`logs/tmp/z_body_replay_20260817_v5_lidar_batch_mt2`:

- causal 3-D RMSE: 0.158 m;
- horizontal RMSE: 0.066 m;
- Z RMSE: 0.143 m;
- Z P95 / maximum: 0.295 m / 0.864 m;
- strict 0.20 m acceptance: failed because P95 and sustained exceedance failed.

This is a runnable reference, not a 15 cm-qualified release. The cleanup code
must be replayed on the same bag before it can replace that measured baseline.

Stable default behavior after this cleanup:

- unified window owns the final state;
- FAST-LIO remains a LiDAR preprocessing and point-to-plane frontend;
- IMU, LiDAR, GNSS, optical flow, and visual observations use one-consumption
  queues;
- D435i uses reprojection factors by default;
- online time calibration remains available, while online extrinsic application
  remains frozen;
- dynamic Z gauge is absent;
- barometer fallback, axis information handoff, axis map protection, and RGB-D
  depth geometry factors are disabled by default.

## Archived or experimental behavior

The following code can remain for controlled A/B work, but is not part of the
stable default:

- RGB-D depth geometry factor: experimental replacement for visual
  reprojection, not an additional factor;
- axis-specific LiDAR information handoff: disabled after the v7/v8 failures;
- barometer local relative-Z fallback: disabled until the pressure topic,
  activation boundary, and segment reset are validated end to end;
- online shared RGB-D/LiDAR map: visualization and mapping side path only; it
  must not feed the same depth observations back into the estimator;
- loop closure and relocalization: experimental recovery path, not a continuous
  duplicate pose constraint;
- online extrinsic calibration candidates: diagnostic only; measured/fixed
  extrinsics remain authoritative.

Rejected experiments:

- v6 dynamic Z gauge improved Z RMSE to 0.087 m but still failed Z P95
  (0.204 m) and sustained-error acceptance;
- v7 weak-Z handoff and v8 axis handoff produced about 5.1 m maximum vertical
  error and are not valid baselines.

## Next validation sequence

1. Build and run all affected unit tests after this cleanup.
2. Replay the frozen v5 bag with the cleaned default configuration.
3. Verify factor accounting: for every modality, selected observations equal
   accepted plus explicitly rejected/superseded observations, and D435i
   reprojection plus RGB-D-depth insertions never exceed visual batches.
4. Fix latest-frame scheduling and output-age spikes before adding new factors.
5. Improve LiDAR Z observability and visual geometry admission independently.
6. Only then enable one experimental Z aid at a time and require causal Z RMSE,
   P95, maximum sustained error, solver latency, and output age to improve.

The target remains less than 0.15 m causal error without using Gazebo truth in
the estimator or mission feedback. No current run has yet proved that target.
