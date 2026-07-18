# BDS/GNSS Integrity and Re-anchor Report

Status: project-owned state-machine core validated offline; no ROS correction output is enabled.

## Implemented boundary

`uf_aiding.SmoothGnssReanchor` consumes local GNSS position, a continuity reference, fix validity, and degradation score. It does not read Gazebo truth or FCU fused local position. The current core implements:

- unreliable-fix and outage rejection;
- jump rejection with no correction output;
- stable-anchor sample collection after outage;
- candidate-anchor stability reset;
- zero-weight re-entry followed by a configurable blend ramp;
- explicit `UNINITIALIZED`, `ACTIVE`, `OUTAGE`, `REANCHORING`, and `REJECTED_JUMP` states.

## Validation scenario

`run_gnss_reanchor_validation.sh` simulates normal operation, a 3 s outage, a 2 s jump, and recovery with a changed GNSS origin. Results from `20260717_state_machine`:

| Metric | Result |
| --- | ---: |
| Samples | 101 |
| Accepted during outage/jump | 0 |
| First recovered acceptance | 10.8 s |
| First recovered blend | 0.0 |
| Maximum accepted position error | 0.056 m |
| Final state | ACTIVE |
| Final blend | 1.0 |

Unit tests also verify that alternating unstable recovery anchors never unlock the aiding output.

## Non-claims and next gate

The core currently accepts local Cartesian inputs. A live ROS node is intentionally deferred until the LLA-to-local conversion uses a documented GeographicLib origin and the output message includes state, blend, covariance inflation, and source timestamps. No GNSS value is currently fed into FAST-LIO or another estimator. Thresholds in the class constructor are validation defaults, not flight parameters.
