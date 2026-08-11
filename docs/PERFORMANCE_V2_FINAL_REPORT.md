# Ultra-Fusion performance V2 final report

## Executive result

The estimator hot path is materially faster and all safety/functionality tests
pass, but the complete V2 performance gate is not met: median live RTF improved
only about 2.0% across the six matched scenarios rather than 20%, and aggregate
time-window rejection was 9.62% rather than approximately 5%.  This branch is
therefore a validated optimization candidate, not a final frozen Performance
V2 release.

## Solver, RTF and resources

| Statistic | V1 | V2 | Delta |
|---|---:|---:|---:|
| six-run solver median | 58.847 ms | 42.399 ms | -27.95% |
| rectangle solver median | 51.973 ms | 33.789 ms | -34.99% |
| S-curve solver median | 59.794 ms | 52.059 ms | -12.94% |
| profiled optimize P50 | 52.867 ms | 40.765 ms | -22.89% |
| profiled optimize P95 | 77.503 ms | 71.980 ms | -7.13% |
| six-run RTF median | 0.460813 | 0.470098 | +2.02% |
| rectangle CPU median | 34.324% | 34.806% | +1.40% |
| S-curve CPU median | 33.447% | 38.788% | +15.97% |
| six-run RAM median | 3.202 GiB | 3.237 GiB | +1.09% |

Profiling-off rectangle runs meet the preferred <=40 ms solver target.  The
longer S-curve does not.  CPU did not fall with solver time because Gazebo,
bridges, camera rendering and longer fully-flushed S-curve evidence dominate
the whole-WSL statistic.

## Visual health

Across the six final production runs, 789 of 1112 quality-valid candidates
were solver-accepted (70.95%), above the 65.8% floor.  All solver accepts were
finite and no track rejection occurred.  There were 107 time-window rejects
(9.62% of quality-valid candidates), a small regression from the combined V1
sample and above the requested approximate 5% goal.  No threshold was widened,
timestamp rewritten, integrity check disabled, or future state used to improve
that number.

The cadence scan selected `balanced`: light/balanced/plus accepted 61/78/85
factors with ATE 0.825/0.532/1.137 m and solver 34.716/37.550/38.962 ms.
`balanced` retained the strongest accuracy/information tradeoff; accepting more
factors was not the objective.

## Accuracy and stability

Rectangle ATE was 0.120/0.487/0.648 m.  Translation RPE was
0.0286/0.0458/0.0417 m and rotation RPE 0.1187/0.1304/0.1279 degrees.  Its
median translation RPE is 34% above the V1 rectangle median and is a warning.

S-curve ATE was 0.477/1.446/0.642 m.  Translation RPE was
0.0500/0.0579/0.0442 m and rotation RPE 0.1117/0.1326/0.1217 degrees.  The
S-curve median translation RPE improved 14.6%; r72 is an ATE outlier and was
retained.  Across all six runs, median ATE improved about 11.7% and median
translation RPE improved about 2.8%, so there is no all-scenario systematic
accuracy regression, but the rectangle warning prevents an unconditional
freeze recommendation.

First unified odom appeared at 62/62/52 s for rectangles and 55/57/52 s for
S-curves.  Startup was not optimized by changing observability gates.

## Joint map regression

The final joint run produced 114204 voxels: 104091 LiDAR voxels, 22444 RGB-D
voxels, 12331 joint voxels and 10113 supplementary RGB-D voxels.  Color coverage
was 11.85%, occupied-volume growth 9.72%, conflict ratio 0, and evictions 0.
LiDAR remained the geometry authority.  The map, unified odom and five sensor
factor paths ran together with zero optimization errors, integrity rejects or
rollbacks.

## Verification

- 15 ROS packages built with `RelWithDebInfo`.
- Full colcon result: 57 tests, 0 errors, 0 failures, 0 skipped.
- Backend 158/158, visual 4/4 and mapping 6/6 direct tests passed.
- D435i active-run lifecycle short test passed.
- Python 198, YAML 29, XML 15 and shell 53 static/syntax checks passed.
- `git diff --check` passed; no live simulation/ROS process, listening flight
  port or active marker remained.  Historical PID files remain only inside
  ignored evidence directories.

## Freeze decision

Do not label this commit as the final frozen Performance V2.  It is suitable as
the next local optimization baseline because its changes are tested, exact and
reversible, but a freeze requires either (a) resolving the WSL GPU/rendering
environment and repeating the matched RTF benchmark, and (b) bringing visual
time-window rejection back near 5% without relaxing the 0.065 s gate, plus one
repeat rectangle set to clear the translation-RPE warning.
