# HybridFusion map-level fusion validation

## Scope and reproducibility

Validation was performed on 2026-08-05 from the unchanged PR #6/PR #8 baseline
`bbe5fe1f068c09244e25e6ce7e80165c05d57928`. Generated maps and run artifacts
are intentionally ignored by Git under `logs/hybridfusion/formal_20260805`.
The committed generator recreates the input deterministically with seed
`20260805`:

- 30 front-and-side RGB-D keyframes along a building-scale route;
- main building, annex, ground, roofs, facades, curb and columns;
- 20,953 visual and 37,539 LiDAR points with different visibility, density and
  noise models;
- true LiDAR-to-visual pose
  `[1.2, -0.8, 0.22, 1 deg, -1.5 deg, 8 deg]`;
- coarse pose `[0.86, -0.46, 0.10, 0 deg, 0 deg, 4.5 deg]`;
- visual map SHA-256 `8f66f9c4888e905a58ac44e0bd949100fd9c23c9f914ff76d9ee35f6af4a41a3`;
- LiDAR map SHA-256 `d5d03e31d274bbf6ca30f3c576898ca051835aee967108dcff7cf1941e8c1de9`.

This is generated simulation data, not a measured Gazebo or hardware result.
It is used because the task explicitly permits automatic generation and it
provides exact SE(3) truth. The opt-in live collection wrapper is separately
syntax/lifecycle checked and reuses the existing headless stack and guided
rectangle without modifying either baseline package.

## Three-method, three-run result

Every method was executed in a fresh process three times; all nine results are
included. `+/-` is population standard deviation, not cherry-picked spread.

| Method | Converged | Translation error (m) | Rotation error (deg) | Overlap NN mean (m) | Boundary error (m) | Inlier ratio | Voxel supplement growth | Runtime (ms) | Peak RSS (MiB) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Coarse pose | 3/3 | 0.49558 +/- 0.00000 | 3.94849 +/- 0.00000 | 0.34066 +/- 0.00000 | 0.54381 +/- 0.00000 | 0.47412 +/- 0.00000 | 1.35555 +/- 0.00000 | 149.00 | 56.42 |
| GICP baseline | 3/3 | 0.00281 +/- 0.00000 | 0.02352 +/- 0.00000 | 0.17504 +/- 0.00000 | 0.45368 +/- 0.00000 | 0.67105 +/- 0.00000 | 1.18654 +/- 0.00000 | 18619.22 | 62.70 |
| HybridFusion | 3/3 | 0.08376 +/- 0.08027 | 0.31432 +/- 0.11152 | 0.20209 +/- 0.02761 | 0.46378 +/- 0.00897 | 0.66895 +/- 0.00230 | 1.14139 +/- 0.03084 | 142349.90 | 61.73 |

HybridFusion per-run translation errors were `0.03356`, `0.02069`, and
`0.19703` m; rotation errors were `0.42545`, `0.16185`, and `0.35567` degrees.
Successful/failed local blocks were `16/2`, `15/3`, and `16/2`. Thus it was
3/3 convergent but less stable and substantially slower than the GICP baseline
on this deterministic scene. GICP is the accuracy winner here. HybridFusion
still improves the coarse pose by 0.412 m in mean translation error, lowers the
overlap mean by 0.139 m, and raises the inlier ratio by 0.195.

The HybridFusion fused map adds occupied voxels equal to 114.14% of the visual
map's count, representing complementary LiDAR ground/facade coverage. The
coarse result's larger 135.55% growth is inflated by misalignment and is not
reported as better map quality. The 118.65% GICP result is the best-aligned
supplement comparison in this dataset.

## Implementation/validation iterations

1. The first implementation passed planar zero-Z boundary clouds to PCL 3D
   NDT and reproducibly crashed in `KdTreeFLANN::radiusSearch`. No result was
   accepted.
2. A dedicated SE(2) Gaussian-grid NDT fixed the singularity, but registering
   every duplicate descriptor pair exceeded 8 minutes. The owned process was
   stopped after 523 seconds and produced no accepted output.
3. Neighborhood consistency now selects the highest combined descriptor score
   per source block and applies a configured cap of 18 local registrations.
   This leaves the paper's explicit ESF correlation threshold at 0.60. The
   formal matrix then completed 9/9 processes, with HybridFusion 3/3.

Failed blocks are real local 2D/3D registration rejections caused by insufficient
boundary support, non-convergence, or failure to meet configured local fitness;
they are counted instead of converted to successes by lowering thresholds.

## Isolation and regression contract

- Neither source PCD is modified; aligned and fused clouds are new files.
- The offline executable publishes no topic or TF and records
  `published_as_tf: false`.
- Both launch files default to `enabled:=false`; disabled exporters subscribe to
  no source data.
- The live wrapper refuses an existing lifecycle marker and cleans up only its
  owned process groups.
- No PR #6 backend, FAST-LIO, GNSS, optical-flow, D_V, RTAB, flight or TF code is
  changed. Full build/test and lifecycle evidence is recorded in the final task
  report associated with this commit.
