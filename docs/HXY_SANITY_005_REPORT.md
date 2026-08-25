# HXY-SANITY-005: backend and native LiDAR correctness audit

Date: 2026-08-25

## Outcome

One current-version correctness defect was reproduced: the prediction recovery
floor could re-admit a LiDAR factor that had already failed the explicit
FAST-LIO/backend frame-consistency gate.  Full local point-plane rank does not
prove global map alignment.  A synthetic full-rank factor with a 2 m map jump
moved a correctly anchored backend state by 1.923 m despite the configured
recovery weight and inflation.

The minimal fix keeps every failed prediction-gate factor disabled.  Every new
scan is still evaluated independently; a scan whose innovation returns inside
the gate is immediately admitted and clears the consecutive-rejection count.
This removes bad-factor reinjection without creating a latch and without
changing the thresholds, Z policy, fusion state machine, relocalization, or
marginalization.

## Audit status

| Item | Status | Evidence |
|---|---|---|
| Backend/FAST-LIO first frame, origin, reset epoch | PASS | First consumed frozen-bag factor is scan 131 at 30.723 s, `camera_init -> body`, reset counter 0. Backend seeds pose and origin directly from that native linearization pose and adds the first prior at the same state. The optimized first pose differs by only 1.5 mm after legitimate residual reduction. Existing reset tests verify transformed pose/velocity, unchanged biases, incremented reset counter, old-buffer flush, and stale/future epoch rejection. |
| Native LiDAR geometry and normal convention | PASS | Frozen-bag raw geometry reproduces exporter residuals within `6.62e-12`, `J^T J` within `8.69e-10`, and `J^T r` within `2.75e-9`. A new right-local finite-difference test agrees within `2e-8`; an isolated factor reduces residual norm from 2.425 to `3.53e-14`. |
| Prediction gate and recovery | FAIL, FIXED | Before the fix, a full-rank but globally shifted factor was admitted on rejection 3 and moved the state 1.923 m. The regression test fails on the old behavior and passes after recovery-floor removal. Existing healthy-next-scan recovery test proves there is no permanent latch. |
| IMU stationary initialization and propagation | PASS | Tilted stationary initialization recovers accel/gyro bias to `1.10e-14`/`1.59e-17`; one-second propagation has position drift `6.34e-15` m, velocity drift `1.32e-14` m/s, and residual norm `1.78e-15`. Analytic manifold IMU Jacobians also pass their finite-difference tests. |
| First abnormal transaction diagnostics | PASS | Transaction/scan/time, innovation, gate reason, recovery flag, effective weight, solver admission, state commit, optimized state, eigenbasis, active factors, and marginalization are present. They identify the first reject at transaction 331 and the first unsafe recovery at transaction 333 without truth entering the estimator. Scan sequence links the trace back to the exact frozen-bag factor for geometry checks. |

## First frame and coordinate contract

The native packet is an absolute unary point-plane factor in FAST-LIO's
`camera_init` map, not a relative odometry factor.  Its state is the body/IMU
pose.  A LiDAR point is transformed as

`p_map = R_map_body (R_body_lidar p_lidar + t_body_lidar) + t_map_body`.

The frozen bag reports `t_body_lidar = [0.05, 0, 0.10]` and a +15 degree
LiDAR-to-body pitch.  Recomputing the first available valid packet from raw
points, extrinsics, map normals, and map plane points reproduces its exported
residual and normal equation at numerical precision.

The backend native-factor trigger ignores the separate FAST-LIO odometry topic.
For the first state it uses the native factor's absolute linearization pose,
sets `lio_origin` to the same position, initializes velocity to zero, seeds
stationary IMU biases when observable, and puts the first prior on that exact
15D state.  The first valid packet in the bag is scan 122, but startup waits for
an observable IMU interval; scan 131 is the first consumed transaction.  This
wait does not create a new coordinate origin.

The supported restart contracts are safe:

- a coordinated process restart starts both sequence histories empty;
- a backend relocalization increments the reset epoch, flushes old buffers,
  drops the in-flight old-epoch factor, and accepts only matching future epochs;
- legacy independent FAST-LIO factors are transformed by `map_from_lio`.

An independent FAST-LIO process-only hot restart is not a supported continuous
operation: its sequence rollback is rejected rather than silently accepted.
This preserves coordinate safety but requires coordinated node restart for
availability.

## Native factor convention

FAST-LIO exports columns in the order map translation, body right-rotation,
LiDAR-to-body right-rotation, and LiDAR-to-body translation.  The backend
validates the exact first six labels and fixes extrinsics, so its 15D order is:

`position[0:3], right-SO(3)[3:6], velocity[6:9], accel_bias[9:12], gyro_bias[12:15]`.

For raw correspondence factors, only columns 0 through 5 are populated.  The
rotation derivative is the body-right perturbation
`(R p_body) with R <- R Exp(delta_theta)`.  Exported `H = J^T J` and
`g = J^T r` are unweighted geometric normals; measurement variance, robust
weight, scheduler weight, and inflation are applied exactly once by the
backend.  The manifold raw path relinearizes the retained correspondences at
the current state.  The condensed fallback consumes the exported right-local
6D normal.  Additive RPY conversion is confined to the legacy linear path.

## Prediction recovery defect

The gate documentation and implementation disagreed.  The gate correctly
classified an excessive innovation as a map/frame consistency fault, but after
three consecutive rejects it allowed `recovery_geometry_usable` to override the
fault.  That flag checks only correspondence validity and local 6D rank.

In HXY-KERNEL-004 C, the first hard reject is transaction 331, scan 461,
`t=63.723 s`, at 1.026 m innovation.  Transactions 333 onward then inject
recovery factors at effective weight `0.2 / 5 = 0.04`, even while innovation
grows from 1.148 m to more than 1.6 m.  There are 107 such factors in that run.

After the fix, the same frozen replay receives all 678 native packets with no
queue overflow, supersession, or latest-only skip.  Recovery factor count is
zero.  Scan 461 remains the first reject and no later globally inconsistent
factor is admitted.  On the common scoreable horizon through 65.69 s, XY RMSE
changes from 1.584 m before the fix to 1.408 m after it.  This comparison is
diagnostic only because callback backlog limits the repaired run's published
horizon.

The correction does not explain the initial horizontal drift: the first gate
reject occurs after drift is already established.  It removes a real secondary
positive-feedback path that amplified the failure after transaction 331.

## IMU audit

The FCU accelerometer convention is body specific force.  At rest it measures
`R_body_map [0, 0, +9.81] + accel_bias`; propagation applies map gravity
`[0, 0, -9.81]` exactly once.  Startup bias is accepted only with sufficient
sample count/span, low mean angular rate, low gyro variation, gravity-consistent
mean force, and low force variation.  Rotation or vibration causes rejection.

The frozen replay accepted 89 samples over 0.881 s and estimated small finite
biases: accel `[0.00911, 0.00154, 0.00079]` m/s2 and gyro
`[0.000447, -0.000437, -0.000422]` rad/s.  These are consistent with the
synthetic stationary startup and do not indicate an origin or gravity-sign
fault.

## Decision

The actual coordinate, factor, perturbation, state-order, and IMU primitives
are sound.  The unsafe prediction recovery override was a current-version real
root cause for post-gate error amplification and is fixed with a regression
test.  It was not the root cause of the earlier weak-direction drift.

It is safe to continue with HXY-INTERACTION from the repaired commit.  That work
should analyze the pre-gate current-window LiDAR/GNSS interaction and integrity
rollback behavior.  It should not restore a rejected LiDAR factor based only on
local observability.

No Gazebo run and no push were performed.
