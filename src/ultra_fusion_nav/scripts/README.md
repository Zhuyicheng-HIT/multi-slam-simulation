# Script Ownership

This directory will contain thin, fail-fast entrypoints for build, rosbag2 record/replay, baseline flight, offline evaluation, and milestone regression matrices.

Scripts must derive the repository root from their own location, accept external dependency paths through documented environment variables, write generated data under ignored output directories, and return nonzero when an acceptance check fails.

Current entrypoints:

- `run_lio_baseline_experiment.sh`: simple-map fixed-route LIO, trajectory, timing, and optional reliability capture.
- `run_reliability_validation.sh`: ROS 2 healthy/degraded endpoint and evidence-policy validation.
- `run_reliability_sweeps.sh`: 11-level complete-evidence formula monotonicity sweep and plot.
- `summarize_stage23_runs.py`: aggregate repeated LIO and reliability runs into one JSON acceptance record.
- `calibrate_optical_flow_lio.py`: evaluator-only optical-flow/LIO timestamp, axis, and scale cross-check.
- `evaluate_optical_flow_gate.py`: combines the hard Gazebo sensor gate with the LIO cross-check without treating an independently invalid LIO reference as a sensor failure.
- `run_gnss_reanchor_validation.sh`: outage, jump, and smooth recovery timeline validation.
