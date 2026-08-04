# ReliabilityScheduler Implementation Report

## Scope

This milestone implements the factor-wise reliability scheduling layer described
in Ultra-Fusion. It does not claim to reproduce the unpublished estimator source.
The public Ultra-Fusion repository currently distributes binaries and documents
the behavior, while the project-owned implementation remains independently
testable.

## Contract

Input topics are `/reliability/{lidar,gnss,imu,optical_flow,vision}_score`.
The scheduler publishes `/reliability/scheduler_state` with, for every modality:

- normalized degradation score `D`;
- continuous reliability weight `s = 1 - D`;
- `factor_enabled` after threshold hysteresis;
- covariance inflation `min(max_inflation, 1 / max(s, min_weight))`;
- reasons and the global health state.

The state set is `NORMAL`, `DEGRADED`, `RISK`, `RELOCALIZING`, `RECOVERED`, and
`FAILSAFE`. A stale or invalid active modality is treated as `D = 1`. Factor
disable and re-enable thresholds differ, and state transitions use dwell times.

## Configuration

The source of truth is `uf_reliability/config/scheduler_config.yaml`. The full
five-modality profile is the default. Tests or reduced pipelines must explicitly
set `active_modalities`; missing inactive modalities do not change global state.
For an active modality, an invalid or stale score disables only that modality's
factor and contributes `D=1` to scheduling. If at least one other active factor
is valid, the global state is `DEGRADED`; all active factors missing still enters
`FAILSAFE`. This keeps optional sensors from taking down a valid LIO/GNSS path.

The Eq. (15) readiness gate is also enforced before scheduling: each score carries
`observation_count` and `minimum_observation_count`. A factor with insufficient
observations is disabled with reason `insufficient_observations_eq15`.

## Fusion integration

The GPS/optical-flow fusion node consumes scheduler decisions when a fresh state
is available. Disabled factors are not accepted or used for initialization.
Enabled factors apply both continuous weight attenuation and covariance
inflation. If no scheduler message exists, the previous per-score behavior is
retained for staged deployment.

MAVROS `nav_msgs/Odometry` forwards pose and velocity covariance, but it cannot
carry MAVLink `ODOMETRY.quality` or `reset_counter`. A dedicated MAVLink bridge is
still required before those two semantics can be considered complete.

## Verification

Pure tests cover continuous weighting, covariance inflation, stale-score
FAILSAFE behavior, factor hysteresis, recovery dwell/hold, and explicit
relocalization. A ROS integration test publishes all five score streams and
verifies the runtime sequence:

`NORMAL -> RISK -> FAILSAFE -> RECOVERED -> NORMAL -> RELOCALIZING`.

Use `record_scheduler_timeline.py` and `plot_scheduler_timeline.py` to retain the
score-to-decision timeline for fault-injection experiments.
