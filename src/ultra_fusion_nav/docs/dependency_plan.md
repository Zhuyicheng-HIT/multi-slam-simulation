# Dependency Plan

The repository should contain project-owned ROS packages, configurations, patches, and reproducibility scripts. Large upstream source trees, build products, bags, maps, and model weights remain external.

## 1. Dependency Policy

- Pin every source dependency by immutable commit in `dependencies.repos` or a dedicated algorithm dependency manifest.
- Record any local patch as a small patch file or project-owned adapter, never as an unversioned copied source tree.
- Resolve paths from the repository root, ROS package share, `$HOME`, or documented environment variables.
- Build external dependencies in `$MULTI_SLAM_EXTERNAL_DIR` (default proposed: `$HOME/multi-slam-deps`) and keep them outside Git.
- Run `rosdep`, repository verification, unit tests, and a clean-workspace build before a milestone push.

## 2. Immediate Corrections

### FAST-LIO ROS2

Current manifest entry:

```text
URL:     https://github.com/hku-mars/FAST_LIO.git
commit: a4743b095409588842a5b30ddfa27e29d2f99164
```

The commit is the upstream ROS2 merge commit. On 2026-07-17 it was shallow-imported with recursive submodule `ikd-Tree` commit `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`, then built successfully in `$HOME/multi-slam-deps/mid360_ws`. Livox driver commit `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` also built successfully. `tools/fetch_external_sources.sh` uses shallow recursive import so a clean setup does not require the full FAST-LIO history.

Three consecutive fixed-route runs from this workspace passed, followed by a fourth pass after the Stage 3 evidence-policy change. The earlier extracted `$HOME/mid360_flight_ws` remains historical only and is no longer the accepted default dependency. Do not update either dependency to a newer HEAD without a separate regression campaign.

Required project-owned extension: expose per-scan diagnostic data without changing the state estimate:

- accepted match count;
- point-to-plane residual mean/median/P95;
- 6x6 normal matrix or eigenvalues;
- angular/spatial support bins;
- local-map insertion count and quality flags.

This patch must be isolated and tested against the uninstrumented pose output before it is used by `D_L`.

### Livox ROS2 Driver

Keep the existing immutable `livox_ros_driver2` pin until a clean import proves it builds with the selected FAST-LIO ROS2 source. Simulation may use `PointCloud2`; `CustomMsg` remains a compatibility path and should not force fake per-point timing into an instantaneous scan.

### rosbag2

SQLite3 is available and is the required baseline storage format. `ros-humble-rosbag2-storage-mcap` is optional and currently absent. Do not make MCAP a requirement for Stage 1; add it only after the SQLite record/replay test passes.

## 3. Planned ROS Packages

| Package | Build type | First stage | Responsibility |
|---|---|---:|---|
| `uf_interfaces` | `ament_cmake` | 1 | Score, health, GNSS integrity, fault-state, and diagnostic messages |
| `uf_sensor_pipeline` | `ament_cmake` plus Python tools where justified | 1 | Topic normalization, frame/time validation, body crop, fault injection, and bag profiles |
| `uf_reliability` | `ament_cmake` | 3 | Deterministic sensor score estimators and evidence publication |
| `uf_aiding` | `ament_python` | 4 | GNSS/optical-flow admission, outage handling, and smooth re-anchor |
| `uf_scheduler` | `ament_cmake` | 6 | Hysteretic continuous weights, gates, covariance inflation, and state machine |
| `uf_backend` | `ament_cmake` | 7 | Offline fixed-lag/sliding-window estimator and ablations |
| `uf_relocalization` | `ament_cmake` | 8 | Keyframe database, place candidates, registration, and recovery checks |
| `uf_evaluation` | `ament_python` | 1 onward | ATE/RPE, score validation, timelines, plots, and experiment manifests |

Messages should contain `std_msgs/Header`, a bounded/validated score convention, evidence values, and explicit source modality. Avoid a generic untyped key/value message for estimator contracts.

## 4. Backend Choice Gate

Do not install a solver during the sensor and scheduler phases.

At Stage 7, run a small offline spike with the same IMU and relative-pose dataset:

| Candidate | Local availability | Strength | Cost/risk |
|---|---|---|---|
| GTSAM | Not installed; source build likely required | Mature IMU preintegration, factor graphs, fixed-lag patterns, covariance access | External source pin and build maintenance |
| Ceres 2.0 | Available from Ubuntu repositories, not installed | Stable nonlinear least squares and custom residuals | Project must own manifold state, preintegration, marginalization, and covariance logic |

Default decision: prefer GTSAM if a pinned Humble/Jammy build and IMU smoke test pass. Fall back to Ceres only if GTSAM packaging is not reproducible. The decision is made before `uf_backend` implementation and recorded in an architecture decision note.

## 5. Additional Libraries

- Geometry: Eigen and PCL are already installed; use them through system packages.
- Images: OpenCV is already installed; use `cv_bridge` for ROS images.
- Geographic conversion: prefer GeographicLib for LLA/ECEF/ENU rather than handwritten geodesy.
- Evaluation: retain the existing NumPy analyzer, then add `evo` as an optional pinned Python tool if TUM trajectory export is implemented.
- Relocalization: prefer a maintained ROS2/PCL implementation or a small project-owned Scan Context descriptor plus PCL NDT/ICP. Pin any imported implementation and audit its license.
- YOLO: weights are external artifacts with checksum, source URL, license, and class map. A simulator ground-truth mask may be used only as an oracle ablation, never reported as YOLO performance.

## 6. Dependency Gates

An external dependency is accepted only when all conditions hold:

1. Immutable source and license are recorded.
2. Clean clone/import succeeds in a new external directory.
3. Ubuntu 22.04 + ROS 2 Humble build succeeds without copying local binaries.
4. Minimal launch publishes the documented interfaces.
5. Repository verification finds no external source, personal absolute path, large artifact, or generated directory in Git.

## 7. Official Ultra-Fusion Runtime

The official repository was rechecked on 2026-07-17 at Git commit `439c8385dbcd78174a2b98ab454b53ec64c9e7ca`. It advertises ROS 2 Humble binary release `v0.2.2`, but states that the estimator source is not yet public. The binary may be used later as an external behavioral oracle on supported datasets; it cannot replace project-owned source, formula audits, or simulator-specific validation.
