# LiDAR Temporal Contract Analysis

## Scope and evidence

This analysis separates a coherent delay at the FAST-LIO/backend boundary from
an intentionally broken interface contract.  The frozen deterministic replay is
`logs/tmp/robustness_v3_frozen_clock_replay_c`; the complete 26-profile result is
`logs/tmp/robustness_v3_1_lidar_temporal_deterministic/temporal_summary.json`.

The frozen bag contains `FrontendScanRequest` and `NativeLidarFactor`, but it
does not contain raw LiDAR packets or per-point timestamps.  Therefore A1 proves
the coherent temporal contract from the FAST-LIO frontend boundary onward.  It
does not claim a physical packet/deskew tolerance for a real MID360.

## What the old injector changed

The previous `native_lidar/time_offset` profile shifted only the
`NativeLidarFactor` header, scan-begin and scan-end stamps.  It did not shift the
paired `FrontendScanRequest`; it also had no access to packet, per-point,
FAST-LIO frontend or deskew timestamps.  The old `+2 ms` result was therefore an
A2 interface-mismatch experiment, not a physical sensor-delay result.

The V3.1 injector now supports two explicit scopes:

- `coherent_frontend_contract`: shifts `FrontendScanRequest`, factor header,
  scan begin and scan end together.
- `factor_only`: shifts only the factor-side stamps to deliberately violate the
  request/factor cache contract.

The scan-request publisher uses reliable, transient-local QoS, matching the
backend subscription.  Timestamp values are shifted; messages are not
restamped with arrival time.

## Deterministic matrix

Each value below used the same frozen messages and ordering.  Optimization
errors, integrity rejects and transaction rollbacks were zero in every row.

| Offset | A1 coherent boundary | A2 factor-only mismatch |
|---:|---|---|
| 0 ms | PASS, completeness 1.000 | PASS, completeness 1.000 |
| ±0.5 ms | PASS, completeness 1.000 | PASS, completeness 1.000 |
| ±1 ms | PASS, completeness 1.000 | PASS, completeness 1.000 |
| ±2 ms | PASS, completeness 1.000 | FAIL: +2 ms has a 1.023 s gap; -2 ms completeness 0.0186 |
| ±5 ms | PASS, completeness 1.000 | FAIL, effectively no usable trajectory |
| +10 ms | PASS, completeness 1.000, max gap 0.627 s | FAIL |
| -10 ms | PASS, completeness 1.000, max gap 0.232 s | FAIL |
| +20 ms | FAIL, completeness 0.6637, first gap 1.716 s | FAIL |
| -20 ms | FAIL, completeness 0.7696 | FAIL |

The demonstrated coherent boundary range is therefore **[-10 ms, +10 ms]**.
The demonstrated factor/request mismatch range is only **[-1 ms, +1 ms]**.
These are tested bounds, not interpolated or universal hardware limits.

## Scan-prediction cache mechanism

At nominal coherent timing, 575 Native factors were produced with 574 cache
hits and zero misses.  Coherent +10 ms retained 542 hits and zero misses; the
backend rejected 43 reuse attempts and 34 scans whose available IMU/window
coverage was no longer suitable.  Coherent +20 ms still had zero cache misses,
but reuse and scan rejection rose to 160 and 202, producing the real loss of
completeness at this interface boundary.

For factor-only +2 ms, the request remains at its original timestamp while the
factor key moves.  At +2 ms this produced 184 reuse rejections and a 1.023 s
trajectory gap.  At -2 ms only 7 factors survived; deferred/released and reuse
diagnostics show the request/factor pairing contract collapsing.  At ±5 ms and
beyond only the initial factor survived.  This is
`INTERFACE_CONTRACT_SENSITIVITY`.

## Online calibration and remaining hardware evidence

The existing LiDAR-IMU time calibration path is shadow-only in this runtime and
the frozen bag lacks an independently shifted raw LiDAR motion stream.  It
cannot legitimately recover or validate packet/point/deskew delay here.  The
following remain `HARDWARE_DATA_REQUIRED`:

- MID360 packet and per-point timestamp propagation;
- scan begin/end derivation in the production driver;
- real deskew trajectory and FAST-LIO frontend timing;
- recovery by the production LiDAR-IMU online time-calibration path.

## Conclusion

The reported `+2 ms` failure was not physical temporal sensitivity.  It was a
deliberate mismatch between `NativeLidarFactor` and its paired trajectory/cache
request.  The interface is now explicit, testable and diagnosed without
changing any association tolerance.
