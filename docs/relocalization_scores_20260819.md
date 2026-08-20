# Relocalization experiment scores

Performance score is 100 points: success 35, accuracy 35, recovery latency 15, motion cost 10, and safe completion 5. Evidence confidence is reported separately and never increases the score.

| Scenario | Logic | Runs | Success | Mean | Minimum | Confidence | Candidates |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- |
| fast | figure8_first_lobe | 1 | 100% | 91.0 | 91.0 | screening_only | 9 |
| nominal | figure8_first_lobe | 1 | 100% | 93.4 | 93.4 | screening_only | 11 |
| nominal | yaw_scan_45deg | 1 | 100% | 87.8 | 87.8 | screening_only | 6 |
| nominal | bias_requested_fallback | 1 | 100% | 87.0 | 87.0 | screening_only | 7 |
| nominal | passive_stationary_zero | 1 | 100% | 85.9 | 85.9 | screening_only | 7 |
| nominal | circle_quarter | 1 | 100% | 80.6 | 80.6 | screening_only | 13 |
| nominal | passive_rotate_preserve | 1 | 100% | 65.0 | 65.0 | screening_only | 12 |
| structural | figure8_first_lobe | 1 | 100% | 88.2 | 88.2 | screening_only | 3 |

## Runs

| Scenario | Logic | Score | RMSE m | P95 m | Endpoint m | Recovery s | Motion m/s | Applied init |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| fast | figure8_first_lobe | 91.0 | 0.0303 | 0.0424 | 0.0290 | 13.49 | 0.60/3.90 | rotate+preserve |
| nominal | figure8_first_lobe | 93.4 | 0.0290 | 0.0451 | 0.0284 | 13.70 | 0.60/4.00 | rotate+preserve |
| nominal | yaw_scan_45deg | 87.8 | 0.0309 | 0.0474 | 0.0192 | 26.77 | 0.00/5.30 | stationary_zero+preserve |
| nominal | bias_requested_fallback | 87.0 | 0.0920 | 0.1218 | 0.1166 | 7.05 | 0.00/0.00 | not_recorded+not_recorded |
| nominal | passive_stationary_zero | 85.9 | 0.0961 | 0.1236 | 0.1245 | 7.81 | 0.00/0.00 | not_recorded+not_recorded |
| nominal | circle_quarter | 80.6 | 0.0807 | 0.1200 | 0.1204 | 15.94 | 0.85/4.90 | stationary_zero+preserve |
| nominal | passive_rotate_preserve | 65.0 | 0.2388 | 0.3547 | 0.2702 | 3.72 | 0.00/0.00 | not_recorded+not_recorded |
| structural | figure8_first_lobe | 88.2 | 0.0452 | 0.0665 | 0.0420 | 19.01 | 0.60/4.00 | stationary_zero+preserve |

A one-run result is only an initial screening signal. Scenario-specific recommendations require repeated runs and must not be inferred from the aggregate score alone.
