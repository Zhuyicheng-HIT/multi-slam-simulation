# DYN-MAP-006: Long-term static map refinement

## Scope and safety contract

This stage starts from `dyn-integration-005-clean-gateway-20260820` and adds a
project-owned, opt-in long-term map downstream of observer v2 and the Clean
Scan Gateway. It does not remap or modify production `/livox/lidar`, FAST-LIO,
the five-source backend, ExternalNav, EKF3, Z-axis fusion, or the PR #14 frozen
tag. The node is disabled by default, publishes no TF, and is not an estimator
input.

All state updates are causal. A scan may use only its scored point cloud, the
strictly previous FAST-LIO posterior, and current/past IMU evidence already
validated by the Clean Gateway. Gazebo truth is used only by the deterministic
evaluator. Missing or stale previous state, timestamp regression, or an
internal input rejection holds the last valid map and emits a degraded status;
it never alters the production LiDAR stream.

## Interface with the Clean Gateway

```text
/livox/lidar (unchanged)
        |
        +--> production Raw FAST-LIO / five-source backend (unchanged)
        |
        +--> observer v2 --> scored world cloud
                              + strictly previous Clean FAST-LIO posterior
                                      |
                                      v
                         long_term_static_map_node (opt-in)
                                      |
                  STATIC_CONFIRMED-only project map products
```

The refinement consumes `/dynamic_observer/scored_cloud` and
`/clean_fast_lio/previous_state`. It publishes only project-owned products:

- `/mapping/long_term_static/points`
- `/mapping/long_term_static/relocalization_points`
- `/mapping/long_term_static/loop_closure_points`
- `/mapping/long_term_static/status`

The relocalization and shared-map configurations are explicit overlays. With
those overlays absent (the default), the existing `/lio/local_map`, shared
mapping input, and all production behavior remain unchanged.

## Voxel lifecycle

Each 0.25 m voxel has one of five reversible states:

```text
UNKNOWN <-> STATIC_CANDIDATE <-> STATIC_CONFIRMED
   ^              |                    |
   |              v                    v
   +---- DYNAMIC_CANDIDATE <-> DYNAMIC_CONFIRMED
```

Static admission requires repeated occupied support, elapsed persistence,
multiple view bins, and an occupied-consistency ratio. The default near-range
contract is 2 observations for candidacy, then 6 observations over at least
1.0 s from at least 2 view bins, with consistency at least 0.65. A single
observer `STATIC` label is therefore never enough for permanent admission.

Far sparse returns (beyond 12 m) require 60 observations over 15 s and 6 view
bins. Insufficiently observed far geometry remains `UNKNOWN`; it is not forced
into either the static or dynamic map merely to improve recall.

Dynamic evidence is likewise accumulated. Three measured free-ray traversals
create a dynamic candidate; six traversals from at least two view bins over
0.4 s confirm it. Two observer dynamic labels can also confirm an observed
dynamic element. Recovery requires 12 static observations over 2 s. All
candidate and confirmed transitions remain reversible.

## FreeDOM-style refinement

For every valid measured return, the implementation accumulates occupied
support at the endpoint and traces only the actually measured ray segment.
Endpoint guard voxels are excluded from free evidence. The map separately
tracks occupied support, static/dynamic labels, free-ray traversals, view masks,
temporal persistence, and vacated-surface evidence.

An old occupied element can be demoted and eventually removed only when later
valid rays physically traverse its voxel. A missing MID360 return, temporary
occlusion, or an unobserved FoV region is not treated as free space. Tests cover
both short and 500-scan occlusion intervals and confirm that a supported static
surface is retained without contradictory rays.

Memory is explicitly bounded by `map.max_voxels` (default 1,500,000). Under
capacity pressure, only stale, non-confirmed elements older than 1,200 scans
are evicted. `STATIC_CONFIRMED` geometry is never aged out solely because it is
unobserved. If the map is still full because it contains only confirmed
geometry, a new candidate is rejected and `capacity_rejected_voxels` is
reported. This preserves the hard memory bound without silently deleting
trusted structure.

## Three-map deterministic evaluation

The long-horizon matrix contains 11 weak scenarios, three deterministic seeds
per scenario, and 280 frames per run. It compares:

- Raw: unfiltered accumulated returns;
- Clean: Clean Gateway-admitted returns;
- Refined: `STATIC_CONFIRMED` output after long-term refinement.

Truth never enters the detector or state machine. It is used only after each
run to score map purity and completeness.

| Scenario | Raw contamination | Clean contamination | Refined contamination | Refined static completeness | Removed ghost voxels |
|---|---:|---:|---:|---:|---:|
| Person stays then leaves | 14.55% | 8.74% | 0.54% | 97.52% | 52 |
| Repeated passes | 61.33% | 1.34% | 0.06% | 97.52% | 14 |
| Multiple crossing | 62.84% | 1.28% | 0.00% | 97.52% | 14 |
| Opening/closing door | 24.20% | 5.39% | 0.58% | 92.71% | 53 |
| Occlusion/reappear | 22.20% | 0.65% | 0.15% | 96.47% | 24 |
| Small fast target | 25.10% | 2.30% | 0.00% | 97.52% | 14 |
| Slow target | 24.19% | 5.58% | 0.84% | 97.52% | 23 |
| Near-wall motion | 57.53% | 1.67% | 0.00% | 97.40% | 14 |
| Large dynamic occlusion | 80.76% | 32.24% | 16.57% | 97.34% | 15 |
| Far sparse target | 23.23% | 23.23% | 0.00% | 96.57% | 19 |
| Stopped then moves | 61.92% | 6.88% | 1.09% | 96.93% | 43 |
| **Macro mean** | **41.62%** | **8.12%** | **1.80%** | **96.82%** | **26** |

The refined ghost ratio is 1.80%, static preservation is 96.82%, and false
removal is 3.18%. Mean admission delay is 2.82 s and mean convergence time is
19.85 s. These metrics do not claim closed-loop FAST-LIO improvement: the map
refinement remains downstream and opt-in.

The earlier frozen dual-FAST-LIO Clean Gateway replay remains complementary
evidence rather than being mixed into the lifecycle matrix. On that real replay,
Raw/Clean contamination was 6.54%/0.77%, static completeness was
98.99%/99.97%, ATE was 6.659/6.722 mm, and RPE was 3.310/3.094 mm. DYN-MAP-006
does not replace or reinterpret those results.

## Large-occlusion limitation

Large dynamic occlusion is the remaining weakest case. Refinement lowers
contamination from 80.76% Raw and 32.24% Clean to 16.57% while retaining 97.34%
of static geometry. The residual is deliberate: after the occluder leaves,
some former occupied voxels are not crossed by enough later measured rays.
Deleting them based only on disappearance would confuse occlusion or MID360
coverage gaps with free space and would violate static-structure protection.
The 500-scan occlusion regression confirms that long absence alone does not
delete `STATIC_CONFIRMED` geometry.

## Relocalization and loop-closure admission

Only `STATIC_CONFIRMED` geometry is emitted to the dedicated relocalization and
loop-closure topics. Dynamic, unknown, and candidate voxels cannot become
permanent keyframe geometry. The offline candidate evaluator reports 1.80%
contamination and 96.82% overlap for the refined admission map, versus 8.12%
contamination for Clean alone. This is a map-admission result, not a claim that
a complete online loop closure was executed.

The opt-in relocalization overlay changes only `keyframe_cloud_topic`; the
opt-in shared-map overlay changes only its long-lived LiDAR source. With
`enabled: false` and without either overlay, no existing relocalization or
shared-map subscription is changed.

## Semantic auxiliary

An optional D435i semantic PointCloud2 input accepts
`x/y/z/dynamic_confidence`. It is disabled by default and, when enabled, is
shadow-only by default. Shadow mode records agreement without changing voxel
state. Geometry remains class-agnostic, and semantic evidence is never a
required condition for removal. A ROS smoke test verifies the shadow path.

## Runtime cost

Across the final 33 deterministic runs, update latency was:

- P50: 0.876 ms
- P95: 1.041 ms
- P99: 1.152 ms
- approximate state memory: 7.76 MiB

The accelerated benchmark saturates roughly one CPU core while processing data
as fast as possible (96.6% measured process CPU). At the intended 10 Hz update
rate, the measured 0.876 ms median update corresponds to approximately 0.88%
of one core, excluding ROS serialization and scheduling. The node uses bounded
state and a bounded previous-state history; no queue overflow or pose timeout
was observed in the smoke test.

## Validation and limitations

Validation covers the 11-scenario/3-seed matrix, the state-machine suite,
long-occlusion and capacity-bound regressions, ROS2 scored-cloud/previous-state
smoke, semantic shadow smoke, offline relocalization admission, full workspace
build/test, syntax checks, and `git diff --check`.

Known limitations and production blockers are:

- the large-occlusion residual described above;
- no claim of a complete online loop closure in this stage;
- frozen replay and deterministic simulation do not replace MID360 hardware
  validation of ray coverage, timing, CPU load, memory growth, and long-run map
  purity;
- production use must validate extrinsics, per-point timing, causal previous
  posterior handoff, and relocalization overlap on team-collected data;
- the feature remains opt-in until those hardware checks pass.

Subject to those production blockers, the software promotion result is
`PROMOTE_LONG_TERM_STATIC_MAP` as a default-off integration candidate.
