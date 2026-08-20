# Relocalization experiment scores

Performance score is 100 points: success 35, accuracy 35, recovery latency 15, motion cost 10, and safe completion 5. Up to 15 points are then deducted for backend integrity events. Evidence confidence and relocalization-candidate and whole-system deployment eligibility are reported separately.

| Scenario | Logic | Runs | Success | Mean | Minimum | Reloc eligible | Deploy eligible | Confidence | Candidates |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| early_route_ambiguity | figure8_first_lobe | 1 | 100% | 73.2 | 73.2 | 0% | 0% | screening_only | 3 |
| fast | figure8_checkpoint4_rotate | 2 | 100% | 91.1 | 89.9 | 100% | 100% | screening_only | 6,7 |
| fast | figure8_checkpoint4_stationary | 2 | 100% | 85.2 | 84.7 | 50% | 50% | screening_only | 6,7 |
| fast | hold_checkpoint8 | 1 | 100% | 64.1 | 64.1 | 0% | 0% | screening_only | 9 |
| fast | figure8_checkpoint8 | 1 | 100% | 50.0 | 50.0 | 0% | 0% | screening_only | 10 |
| nominal | stationary_velocity_bias_requested | 1 | 100% | 84.0 | 84.0 | 0% | 0% | screening_only | 7 |
| nominal | figure8_first_lobe | 1 | 100% | 78.4 | 78.4 | 0% | 0% | screening_only | 11 |
| nominal | yaw_scan_45deg | 1 | 100% | 75.8 | 75.8 | 0% | 0% | screening_only | 6 |
| nominal | stationary_velocity | 1 | 100% | 70.9 | 70.9 | 0% | 0% | screening_only | 7 |
| nominal | circle_quarter | 1 | 100% | 65.6 | 65.6 | 0% | 0% | screening_only | 13 |
| nominal | passive_stationary_zero | 1 | 100% | 50.0 | 50.0 | 0% | 0% | screening_only | 12 |
| structural_window | hold_checkpoint4 | 2 | 100% | 96.1 | 93.8 | 100% | 100% | screening_only | 2,7 |
| structural_window | figure8_checkpoint4_rotate | 2 | 100% | 72.8 | 58.9 | 50% | 50% | screening_only | 4,5 |
| structural_window | figure8_checkpoint4_stationary | 1 | 100% | 58.2 | 58.2 | 0% | 0% | screening_only | 6 |
| window_opening | passive_hold | 1 | 100% | 97.0 | 97.0 | 0% | 0% | screening_only | 9 |
| window_opening | passive_hold_legacy | 1 | 100% | 85.0 | 85.0 | 0% | 0% | screening_only | 11 |
| window_opening | figure8_first_lobe | 1 | 100% | 84.9 | 84.9 | 0% | 0% | screening_only | 2 |

## Runs

| Scenario | Logic | Score | RMSE m | P95 m | Endpoint m | Recovery s | Motion m/s | Reloc integrity | Reloc eligible | Deploy eligible | Applied init | Evidence |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- | --- |
| early_route_ambiguity | figure8_first_lobe | 73.2 | 0.0452 | 0.0665 | 0.0420 | 19.01 | 0.60/4.00 | 18 | no | no | stationary_zero+preserve | explicit_diagnostic;run_cumulative_fallback;duration_complete |
| fast | figure8_checkpoint4_rotate | 92.4 | 0.0347 | 0.0582 | 0.0454 | 13.23 | 0.60/3.90 | 0 | yes | yes | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| fast | figure8_checkpoint4_rotate | 89.9 | 0.0483 | 0.0772 | 0.0386 | 14.21 | 0.60/4.00 | 0 | yes | yes | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| fast | figure8_checkpoint4_stationary | 85.7 | 0.0394 | 0.0626 | 0.0217 | 14.18 | 0.60/4.00 | 2 | no | no | stationary_zero+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| fast | figure8_checkpoint4_stationary | 84.7 | 0.0686 | 0.1087 | 0.1084 | 13.36 | 0.60/3.90 | 0 | yes | yes | stationary_zero+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| fast | hold_checkpoint8 | 64.1 | 0.2594 | 0.5338 | 0.3845 | 6.80 | 0.00/0.00 | 0 | no | no | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| fast | figure8_checkpoint8 | 50.0 | 0.2248 | 0.4526 | 0.3152 | 12.59 | 0.60/3.90 | 3 | no | no | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| nominal | stationary_velocity_bias_requested | 84.0 | 0.0920 | 0.1218 | 0.1166 | 7.05 | 0.00/0.00 | 1 | no | no | rotate+preserve | inferred_legacy_diagnostic;run_cumulative_fallback;duration_complete |
| nominal | figure8_first_lobe | 78.4 | 0.0290 | 0.0451 | 0.0284 | 13.70 | 0.60/4.00 | 6 | no | no | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;duration_complete |
| nominal | yaw_scan_45deg | 75.8 | 0.0309 | 0.0474 | 0.0192 | 26.77 | 0.00/5.30 | 4 | no | no | stationary_zero+preserve | explicit_diagnostic;run_cumulative_fallback;duration_complete |
| nominal | stationary_velocity | 70.9 | 0.0961 | 0.1236 | 0.1245 | 7.81 | 0.00/0.00 | 20 | no | no | stationary_zero+preserve | inferred_legacy_diagnostic;run_cumulative_fallback;duration_complete |
| nominal | circle_quarter | 65.6 | 0.0807 | 0.1200 | 0.1204 | 15.94 | 0.85/4.90 | 8 | no | no | stationary_zero+preserve | explicit_diagnostic;run_cumulative_fallback;duration_complete |
| nominal | passive_stationary_zero | 50.0 | 0.2388 | 0.3547 | 0.2702 | 3.72 | 0.00/0.00 | 24 | no | no | rotate+preserve | inferred_legacy_diagnostic;run_cumulative_fallback;duration_complete |
| structural_window | hold_checkpoint4 | 98.3 | 0.0412 | 0.0635 | 0.0182 | 4.84 | 0.00/0.00 | 0 | yes | yes | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| structural_window | hold_checkpoint4 | 93.8 | 0.0540 | 0.0897 | 0.0387 | 8.43 | 0.00/0.00 | 0 | yes | yes | stationary_zero+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| structural_window | figure8_checkpoint4_rotate | 86.7 | 0.0594 | 0.1086 | 0.0509 | 14.54 | 0.60/4.00 | 0 | yes | yes | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| structural_window | figure8_checkpoint4_rotate | 58.9 | 0.1706 | 0.3032 | 0.2192 | 14.17 | 0.60/3.90 | 1 | no | no | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| structural_window | figure8_checkpoint4_stationary | 58.2 | 0.2811 | 0.5935 | 0.2594 | 14.16 | 0.60/4.00 | 0 | no | no | stationary_zero+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| window_opening | passive_hold | 97.0 | 0.0303 | 0.0487 | 0.0163 | 4.09 | 0.00/0.00 | 1 | no | no | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |
| window_opening | passive_hold_legacy | 85.0 | 0.0287 | 0.0462 | 0.0286 | 4.10 | 0.00/0.00 | 28 | no | no | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;duration_complete |
| window_opening | figure8_first_lobe | 84.9 | 0.0287 | 0.0498 | 0.0182 | 12.71 | 0.60/4.00 | 3 | no | no | rotate+preserve | explicit_diagnostic;run_cumulative_fallback;early_landing |

A one-run result is only an initial screening signal. Scenario-specific recommendations require repeated runs and must not be inferred from the aggregate score alone.

## Current conclusions

- The experiment wrapper requests `stationary_zero` only for `hold`. Active
  `yaw_scan`, `circle`, and `figure8` actions default to `rotate`, while
  preserving IMU bias. Active motion must not be treated as a stationary
  initialization observation.
- In the fast-motion scene, checkpoint 4 plus `figure8 + rotate` scored 92.4
  and 89.9 in two complete runs. Both passed strict acceptance with zero
  post-relocalization optimization/native integrity events. Mean score: 91.1.
- The previous fast-motion `figure8 + stationary_zero` pair scored 84.7 and
  85.7; one run had two post-reset integrity events. This is evidence against
  stationary initialization during active excitation.
- In the structural-window scene, `hold + checkpoint 4` passed twice, scoring
  98.3 and 93.8 (mean 96.1), both with zero post-reset integrity events.
- Structural-window `figure8 + rotate` is not stable: one run passed at 86.7,
  but the repeat scored 58.9 after a native queue discard and failed accuracy
  gates. Its two-run mean is 72.8 and only 50% of runs are eligible. It is a
  fallback experiment, not the structural-scene default.
- The window-opening single-run ranking remains passive hold first. Its score
  is now 97.0 because the refreshed scorer also counts the previously
  unreported cumulative queue integrity event. This older log uses cumulative
  fallback evidence, not a post-reset delta.
- Fast checkpoint 8 remains poor: hold RMSE 0.2594 m and figure-eight RMSE
  0.2248 m. Trigger timing is therefore a first-order factor; checkpoint 4 is
  preferred for the current fast route.

## Recommended policy

1. Use `hold + checkpoint 4` for structural-window or otherwise ambiguous
   scenes when the candidate is already geometrically acceptable.
2. Use `figure8 + rotate + preserve + checkpoint 4` for fast motion or when
   passive registration is insufficient. Do not request `stationary_zero`
   during active excitation.
3. Keep yaw scan and circle as fallback ablations. Their nominal scores after
   the complete integrity accounting are 75.8 and 65.6.
4. Treat the scores as screening/preliminary evidence, not a final deployment
   claim. Fast rotate and structural hold have two complete runs; structural
   active rotate is explicitly not stable in the current two-run comparison.
