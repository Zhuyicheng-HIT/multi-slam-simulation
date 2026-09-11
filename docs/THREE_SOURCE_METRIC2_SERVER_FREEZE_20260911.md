# Three-Source and Metric-2 Server Freeze Report

## Scope and evidence

- Workspace: `/home/ld666/projects/multi-slam-three-source-metric2`
- Pre-closure HEAD: `09a78283ffb64c8f71225f4b0362293915aa1f3d`
- Persistent evidence: `/home/ld666/validation-artifacts/three-source-metric2-20260911`
- Environment: server Gazebo/SITL and deterministic benchmarks; no NUC or real sensor validation.
- Fusion factors, weights, gates, and estimator timing semantics were not changed.

The previously completed A nominal campaign passed three flights at approximately
`0.019 m` XY RMSE, with complete takeoff, rectangle, LAND, and disarm. The
current persistent closure directory contains the new A-medium trials rather
than copies of those earlier nominal run directories. The nominal pass is not
reclassified as new evidence by this report.

## LiDAR degradation contract

The validation-only profile uses fixed seed `731` and defines correspondence
dropout as follows:

| Level | Dropped backend correspondences | Retained correspondences |
| --- | ---: | ---: |
| light | 20% | 80% |
| medium | 60% | 40% |
| heavy | 85% | 15% |

This is degradation of backend NativeLidarFactor correspondence information.
It is not equivalent to a physical scene with weak geometric observability,
weather contamination, altered ray sampling, or a blocked MID360. Heavy may be
combined with an explicitly documented geometry case in a future campaign, but
no such physical-geometry claim is made here.

## A-medium flight results

All three valid runs used the same profile and completed takeoff, four of four
rectangle waypoints, LAND, and disarm. Environment startup failures were kept
as environment evidence and were not counted as trials.

| Run | XY RMSE / P95 / max (m) | Z RMSE / P95 / max (m) | 3D RMSE / P95 / max (m) | longest >0.20 m (s) | LiDAR / IMU / Flow factors | rollback / opt error / queue overflow |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| a-medium-001 | 0.01787 / 0.02983 / 0.04401 | 0.01230 / 0.02437 / 0.03552 | 0.02170 / 0.03350 / 0.04462 | 0 | 602 / 601 / 147 | 0 / 0 / 0 |
| a-medium-002 | 0.01859 / 0.03188 / 0.06095 | 0.01242 / 0.02383 / 0.05484 | 0.02236 / 0.03427 / 0.06100 | 0 | 599 / 598 / 145 | 0 / 0 / 0 |
| a-medium-003r | 0.01806 / 0.03119 / 0.04684 | 0.01213 / 0.02309 / 0.04043 | 0.02176 / 0.03414 / 0.04742 | 0 | 600 / 599 / 135 | 0 / 0 / 0 |

The three-run A-medium result passes the 0.20 m trajectory criterion with no
persistent rollback, solver rejection, or queue overflow. Source rates were
approximately 10 Hz LiDAR, 10.27 Hz flow input, and 9.75 Hz unified odometry.

## Metric-2 scoring contract

No low-scoring frame is deleted. Each reset/repetition retains every evaluated
adjacent frame pair. The first 12 detector-history frames are additionally
labeled `startup_warmup`; all later pairs are labeled `steady_state`. The
official/default score remains `all_frames`. The steady-only result is a
diagnostic supplement because permission to exclude warm-up has not been
established from the competition rules.

For each cell below, repeatability metrics are `micro / P5 / minimum / fraction
below 95%`. Feature counts are retained common-visible static features.

| Scenario | All frames (%) | Warm-up (%) | Steady state (%) | pairs all/warm/steady | features all/warm/steady |
| --- | ---: | ---: | ---: | ---: | ---: |
| static_fast_turn | 96.74 / 71.43 / 0.00 / 12.28 | 88.94 / 0.00 / 0.00 / 30.00 | 99.00 / 93.89 / 80.82 / 5.95 | 228/60/168 | 104438/23488/80950 |
| new_area_in_fov | 96.51 / 71.43 / 0.00 / 10.53 | 88.94 / 0.00 / 0.00 / 30.00 | 98.72 / 95.92 / 78.43 / 3.57 | 228/60/168 | 104110/23488/80622 |
| person_crossing | 97.13 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.99 / 99.76 / 99.75 / 0.00 | 228/60/168 | 90772/23488/67284 |
| stationary_then_moving | 97.13 / 71.69 / 0.00 / 7.89 | 88.97 / 0.00 / 0.00 / 30.00 | 99.97 / 99.75 / 99.48 / 0.00 | 228/60/168 | 91102/23530/67572 |
| multiple_people_crossing | 97.11 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.98 / 99.76 / 99.75 / 0.00 | 228/60/168 | 90174/23488/66686 |
| small_fast_target | 97.13 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.99 / 99.76 / 99.75 / 0.00 | 228/60/168 | 91010/23488/67522 |
| slow_target | 97.12 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.99 / 99.76 / 99.75 / 0.00 | 228/60/168 | 90708/23488/67220 |
| moving_box_or_vehicle | 97.11 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.98 / 99.76 / 99.75 / 0.00 | 228/60/168 | 90230/23488/66742 |
| opening_closing_door | 96.94 / 72.28 / 0.00 / 7.89 | 88.74 / 0.00 / 0.00 / 30.00 | 99.94 / 99.75 / 99.05 / 0.00 | 228/60/168 | 92296/24702/67594 |
| large_dynamic_occlusion | 96.50 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.96 / 99.68 / 99.04 / 0.00 | 228/60/168 | 74788/23488/51300 |
| radial_approach_departure | 97.12 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.99 / 99.76 / 99.75 / 0.00 | 228/60/168 | 90714/23488/67226 |
| moving_then_stops | 97.12 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.99 / 99.76 / 99.75 / 0.00 | 228/60/168 | 90688/23488/67200 |
| co_moving_target | 96.95 / 72.58 / 0.00 / 7.02 | 88.62 / 0.00 / 0.00 / 26.67 | 99.98 / 100.00 / 99.18 / 0.00 | 228/60/168 | 85022/22718/62304 |
| occlusion_appear_disappear | 96.98 / 74.35 / 0.00 / 8.77 | 88.26 / 0.00 / 0.00 / 33.33 | 99.95 / 99.64 / 99.27 / 0.00 | 228/60/168 | 124670/31668/93002 |
| near_wall_motion | 97.13 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.99 / 99.76 / 99.75 / 0.00 | 228/60/168 | 90766/23488/67278 |
| vertical_target_motion | 97.14 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.99 / 99.76 / 99.75 / 0.00 | 228/60/168 | 91174/23488/67686 |
| nonrigid_motion | 97.12 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.99 / 99.76 / 99.75 / 0.00 | 228/60/168 | 90580/23488/67092 |
| far_sparse_target | 97.14 / 71.43 / 0.00 / 7.89 | 88.94 / 0.00 / 0.00 / 30.00 | 99.99 / 99.76 / 99.75 / 0.00 | 228/60/168 | 91220/23488/67732 |

Aggregate V2 results are:

| Window | micro | P5 | minimum | <95% frame ratio | frame pairs | retained features |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| all frames | 97.0054% | 71.6931% | 0% | 8.2846% | 4104 | 1674462 |
| startup warm-up | 88.8622% | 0% | 0% | 30.0000% | 1080 | 431450 |
| steady state | 99.8319% | 99.7487% | 78.4274% | 0.5291% | 3024 | 1243012 |

Therefore Metric-2 does **not** completely pass under the required all-frame
definition. Excluding warm-up yields a strong supplemental steady-state score,
but is not the default or official result.

## Steady-state frames below 95%

Only two scenarios contain steady-state pairs below 95%. Repetitions are
reported separately and intentionally retained even when deterministic inputs
produce identical values.

- `static_fast_turn`: seed 101 repetitions 0/1, frames 21->22 at 82.9918% and
  22->23 at 93.4579%; seed 202 repetitions 0/1, 21->22 at 80.8247%; seed 303
  repetitions 0/1, 21->22 at 82.1782% and 22->23 at 93.8931%.
- `new_area_in_fov`: seeds 101/202/303, repetitions 0/1, frame 21->22 at
  78.9062%, 78.4274%, and 79.8450%, respectively.

Every deficit is attributed to the matched feature being
`unknown_or_unconfirmed` in the previous frame. The diagnostic recorded zero
previous-frame dynamic classifications for these deficits. The transition at
frame 22 changes visibility/turn geometry and temporarily invalidates prior
static confirmation; it is not evidence that Dynamic deleted the current
retained features.

Interactive SVG timelines are stored under:
`/home/ld666/validation-artifacts/three-source-metric2-20260911/metric2/dynamic_observer_benchmark_visualizations`.
Orange points are warm-up failures and red points are steady-state failures.
The exact seed, repetition, frame, score, retained count, and repeatable count
are embedded in each SVG point tooltip. The machine-readable complete list is
in each scenario's `low_repeatability_frames` array in
`dynamic_observer_benchmark.json`.

## Smokes and resource evidence

- Clean Gateway fail-open passed raw passthrough for pose/IMU timeout, queue
  overflow, timestamp regression, and epoch reseed behavior.
- Manual relocalization smoke passed 4 tests covering start, duplicate request,
  cancel, stale timestamp, timestamp regression, source loss, and lease expiry.
- Trial lifecycle smoke passed and now registers the current run's external
  ArduPilot process group only after PID start-time and cmdline validation.
- A-medium server CPU median was 12.1--14.0%, CPU P95 20.5--23.0%, and RAM P95
  3.86--3.91 GiB. These are simulation-host measurements, not NUC margins.
- NUC CPU, RAM, temperature, real MID360 throughput, and real flight validation
  remain pending.

## Verification and release status

- Full build: 22 packages completed.
- Full test execution: 22 packages completed.
- `colcon test-result --verbose`: 269 tests, 0 errors, 0 failures, 0 skipped.
- A first final-test invocation without the required external MID360 overlay is
  retained as `colcon-test-result-missing-external-overlay.log`: it reported one
  import error and one launch failure because `fast_lio.msg` and
  `livox_ros_driver2.msg` were unavailable. Sourcing
  `/home/ld666/multi-slam-deps/mid360_ws/install/setup.bash`, as required by the
  project dependency contract, resolved both without a source change.
- SVG XML parsing and benchmark JSON parsing passed.
- No production algorithm threshold or sample exclusion was introduced.

This is a server-validated candidate only. A formal frozen release tag must not
be created until NUC resource and hardware validation are complete.
