# LIDAR-DIR-001 directional reliability (2026-08-24)

## Baseline and scope

The experiment starts from annotated tag `current-complete-pr14-20260824`
(`054a6744cf2265bd4dc1bd4cee0be6287cd2dbc1`). The synchronized upstream is
PR #14 branch `feat/core-algorithm-cleanup-20260817` at
`5f15ab032949e24b539375b4bfa6349e6b562b3b`. Safety, avoidance, Dynamic,
active relocalization, ExternalNav and Z-axis policy are unchanged.

The latest PR #14 launch envelope is used: body-frame X/Y `[-0.45, 0.45] m`
and Z `[-0.35, 0.15] m`. In the strict rectangle run it removed only about
0.13 percent of points at P95, but all new evidence is kept separate from old
`e934132` geometry results.

## Existing-code audit

- The Reliability Scheduler still produces one scalar LiDAR score from the
  Hessian spectrum, normal diversity, axial penalty and match count. That
  scalar can enable or scale the complete factor.
- Native point-to-plane correspondences already retain the 6x6 normal,
  matched-plane provenance, map-frame coordinates and causal scan timestamp.
- Translation information conditioned on rotation is already computed as
  `H_tt - H_tR pinv(H_RR) H_Rt`.
- `AxisReliabilityProfile`, XYZ diagnostics, conditional XYZ shaping and
  arbitrary eigensubspace shaping already exist in factor math.
- `axis_information_handoff_enabled=false` and
  `subspace_information_handoff_enabled=false` remain the production defaults.
- Existing XYZ production trial is Z-only. The candidate file enables XYZ and
  subspace explicitly, but is not included by any production launch.
- Handoff uses only fresh, valid, independently admitted GNSS/RGB-D/barometer
  information. It does not resurrect hard-rejected sources or add weight in
  the LiDAR-strong complement. One observation remains one factor.
- PR #14 did not change these interfaces or defaults; it changed the geometry
  reaching them through the enlarged body envelope.

## Reliability definition and failure boundary

The diagnostic candidate now reports three distinct terms:

`R_direction = source_health * factor_consistency * geometry_observability`.

Health is binary for contract failures. Consistency is causal innovation
evidence. Observability is the normalized eigenspectrum of the Native
conditional translation information. XYZ support is the map-axis projection
of the same matrix, not a second detector. Expected scene direction is passed
only to the offline scorer.

Dropout, stale input, timestamp regression/future stamps, non-finite values and
contract corruption are `HARD_REJECT`. Healthy corridor, plane, partial-sector
and occlusion geometry remains `ADMIT_DIRECTIONAL`; weak information is not a
reason to disable the whole LiDAR factor.

## Deterministic A/B/C matrix

The frozen-normal benchmark runs five seeded repeats for 13 cases. It uses the
same current Native conditional normal and the existing production shaping
functions. It is not a flight trajectory, so it reports one-step projected
pose error rather than inventing ATE/RPE. Flight ATE/RPE evidence remains the
Phase-0 strict rectangle and tunnel startup runs.

Methods:

- A: conservative scalar geometry score applied to the complete normal.
- B: XYZ conditional information handoff.
- C: conditional eigensubspace handoff.

All angles between the expected weak subspace and the detected subspace are
below 0.000002 degrees. Values below are five-run RMSE in metres.

| Scene | weak angle | A weak | B weak | C weak | A strong | B/C strong |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| normal 3-D room | N/A | 0 | 0 | 0 | 0.0065 | 0.0065 |
| X corridor | 0 | 0.0910 | 0.1480 | 0.1480 | 0.0216 | 0.0066 |
| Y corridor | 0 | 0.0642 | 0.1259 | 0.1259 | 0.0189 | 0.0061 |
| 45-degree corridor | 0 | 0.0604 | 0.2280 | 0.1292 | 0.0230 | 0.0070 |
| floor dominant | 0 | 0.0799 | 0.1358 | 0.0846 | 0.0221 | 0.0053 |
| Z weak | 0 | 0.1046 | 0.1579 | 0.1579 | 0.0199 | 0.0048 |
| single wall | 0 | 0.0893 | 0.1439 | 0.0961 | 0.0177 | 0.0028 |
| partial sector | 0 | 0.1519 | 0.2537 | 0.1362 | 0.0158 | 0.0071 |
| asymmetric occlusion | 0 | 0.1594 | 0.2539 | 0.1654 | 0.0099 | 0.0047 |

Aggregate 3-D RMSE is A/B/C = 0.1015/0.1773/0.1256 m. Aggregate strong-
subspace RMSE is 0.0181/0.0058/0.0058 m. C is materially better than B for a
rotated or multi-axis weak subspace: in the 45-degree corridor, B leaves all
three axis scales at one, while C detects `[0.707, 0.707, 0]`, scales that
eigendirection to 0.25 and reduces B's weak error by about 43 percent.

This is a real Pareto result, not a universal win. Scalar A gives lower total
error in this frozen-normal model because it yields more control to the
independent source, but it needlessly loses LiDAR-strong information. C
preserves strong directions and outperforms B for rotated geometry, yet does
not beat A overall. Therefore the evidence does not justify default-on use.

## Live runtime evidence

The latest-envelope rectangle scalar baseline completed 4/4 legs, LAND and
disarm. Native/IMU/GNSS/flow counts were 1123/1134/571/293, with zero optimizer
errors, rollback or queue overflow. Unified 3-D RMSE/P95/max was
2.45/4.32/6.13 cm and endpoint error was 2.79 cm.

The latest tunnel 3 m straight startup completed 1/1, LAND and disarm with
zero optimizer errors, rollback and overflow. Native LiDAR remained admitted
and its prediction gate reported zero rejection in that short interval, but
horizontal RMSE was 1.57 m and optical flow admitted zero factors. This is a
startup smoke, not a directional A/B/C flight pass. The older B3 fixed replay
cannot close this gate because it predates the new PR #14 geometry baseline.

The deterministic matrix performs no prediction-gate rejection and admits all
45 healthy geometric samples; all 20 hard-failure samples are rejected before
factor assembly. There are no optimizer, rollback or queue operations in this
algebraic benchmark. Directional calculation latency P50/P95/P99 is
0.397/0.605/0.629 ms and process maximum RSS is 57.7 MiB.

## Verification and decision

- full build: 20 packages PASS
- full colcon test: PASS; aggregate ament result 192 tests with zero errors,
  failures or skips; backend suite 322 tests PASS, including five new matrix
  contract tests
- Python compile and `git diff --check`: PASS
- production defaults, safety/command ownership and Raw/Clean ownership are
  unchanged

Decision: **DO_NOT_PROMOTE**.

The candidate correctly separates health from geometry, detects arbitrary
weak directions, preserves strong information, and hard-rejects bad streams.
However, it lacks latest-envelope, identical-input full-trajectory A/B/C for
normal, rotated-corridor and partial-FoV cases; deterministic total error also
does not establish a universal advantage over scalar A. Keep both switches
default-off. Next work should record one latest-PR14 Native-factor dataset per
geometry family, then replay the identical samples through A/B/C while
measuring ATE/RPE, prediction-gate interaction and marginal covariance.
