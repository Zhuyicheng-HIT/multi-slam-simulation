# LiDAR Directional Degeneracy Experiment (2026-08-25)

This document records experimental work for reproducing and mitigating LiDAR
directional degeneracy. The stable defaults remain unchanged: `hybrid` scoring
and `adaptive` admission. The paper-strict path is opt-in and belongs on an
experimental branch until repeated tunnel and ordinary-indoor validation pass.

## Experimental changes

- `paper_eq19` computes the public paper's LiDAR reliability score directly
  from the directional information eigenvalues.
- `paper_eq15` applies binary admission at the configured activation threshold.
  It also disables the stable-path stale-score and anchor-protection overrides.
- The tunnel has visual-only floor checker tiles, a dashed white centerline, and
  asymmetric colored wall panels. They add camera features without collisions,
  so the LiDAR geometry is unchanged.
- The rectangle mission runner has an opt-in `straight` mode that flies one
  positive-Y leg and then lands.
- Long-running observers stop on the `landed` mission phase.
- Expensive shadow diagnostics can be disabled independently for compute
  isolation. Their defaults remain enabled.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
source install/setup.bash
bash tools/run_lidar_degeneracy_tunnel_experiment.sh
```

The wrapper uses Gazebo-truth route feedback and does not feed ExternalNav to
EKF3. It enables the strict paper score/admission path and RGB-D direct factors.

## Textured-tunnel direct-factor run

Log: `logs/lidar_deg_visual_anchor_direct_straight_1ms_final_20260825`

- Mission: takeoff, 2 m straight leg, LAND accepted, disarmed.
- Causal error: 3D RMSE 0.0749 m, P95 0.1399 m, max 0.3700 m; XY RMSE
  0.0402 m; Z RMSE 0.0632 m; endpoint 0.0393 m.
- RGB-D direct: 59 received, 52 attempted, 52 formed and 52 solver-accepted;
  factor/attempt ratio 100%, factor/received ratio 88.1%.
- Optical flow: 404 received, 316 attempted, 177 formed; factor/attempt ratio
  56.0%, factor/received ratio 43.8%. The requested 50% target is met when the
  denominator is backend factor attempts.
- GNSS: 158 received, 157 attempted, 154 formed.
- Native LiDAR packets: 316 received and 316 processed by the front end. These
  counters do not prove solver admission; strict scheduler diagnostics must be
  used for that conclusion.
- Solver P95 2.81 ms; backend callback P95 40.09 ms; unified output age P95
  34 ms.

This run failed only the sustained-error-duration acceptance gate (0.974 s over
the 0.5 s limit), so it is evidence for continued experimentation, not a new
stable accuracy baseline.

## Ordinary-indoor upload gate

Log: `logs/experimental_pr_indoor_gate_20260825`

- Exact branch build completed and all five backend factor types were active.
- Mission completed takeoff, 1 m straight leg, landing, and disarm.
- Causal 3D RMSE 0.0155 m, P95 0.0234 m, max 0.0552 m; XY RMSE 0.0073 m;
  Z RMSE 0.0137 m; endpoint 0.0150 m.
- Optical-flow source validation stopped at landing and passed: scale 0.954,
  normalized RMSE 0.184, correlation 0.984, median quality 163.
- The original checker expected four rectangle waypoints and 300 samples. The
  runner now gives straight missions an explicit one-waypoint, 150-sample
  contract without weakening the rectangle defaults.

## Simulation performance investigation

The sensors retain their existing rates, resolution, field of view, range, and
noise settings.

- Gazebo uses Ogre2 on the NVIDIA D3D12 renderer. MID360 and the flow range
  sensor already use GPU LiDAR; D435 RGB-D and flow camera rendering are also on
  the GPU path.
- Full camera and LiDAR transport without SITL/backend reached about 0.32 RTF.
  LiDAR-only and flow-only transport each reached about 0.46 RTF. This isolates
  rendering/transport synchronization and copies as the dominant ceiling; the
  monitoring stack is not the primary cause.
- GPU utilization remained about 7-8%, while Gazebo consumed more than one CPU
  core. This is consistent with a WSL D3D12/Gazebo synchronization bottleneck,
  not exhausted shader throughput.
- A 2 ms physics step first violated ArduPilot IMU/loop-rate checks. Lowering the
  temporary SITL loop target allowed takeoff but later lost Gazebo sensor JSON
  traffic and disconnected SITL. Both changes were reverted.

The 0.70 RTF target was not reached without changing sensor parameters. Reaching
it likely requires a larger architectural change such as an in-process C++/GPU
bridge that avoids current transport copies; it should not be claimed as solved
by disabling observers.

## Next experiment

Capture the directional information eigenvectors, not only axis-aligned
diagonal scores, and project each sensor's information into the same local
degeneracy basis. Compare `hybrid/adaptive` against `paper_eq19/paper_eq15` one
variable at a time on the same replay. Report solver-admitted factors separately
from received and front-end processed packets.
