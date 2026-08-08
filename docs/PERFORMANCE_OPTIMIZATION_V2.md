# Ultra-Fusion performance V2 changes

## Kept changes

1. Added bounded, opt-in stage profiling for backend callbacks, nonlinear
   factors, process groups and shared-map updates.  Default behavior has no
   sample collection and no per-factor JSON stream.
2. Replaced the dense marginal-prior block-diagonal Jacobian construction and
   `J.T @ H @ J` products with algebraically identical 3x3 rotation block
   transforms.  A direct dense-versus-block equality test protects the change.
3. Replaced transactional `deepcopy` of immutable factor payload arrays with
   copied factor dictionaries.  State objects remain deep copies; an isolation
   test covers index replacement during marginalization and rollback.
4. Vectorized analytic RGB-D reprojection residuals and Jacobians with NumPy.
   Runtime finite differences remain absent; the finite-difference Jacobian
   equality test remains in the test suite.
5. Batched shared-map finite filtering, voxel-index calculation and color
   clipping, while retaining sequential per-voxel confidence and source update
   semantics.
6. Packed PointCloud2 data from structured NumPy arrays instead of constructing
   Python tuple lists.
7. Added `balanced_light` (0.24 s) and `balanced_plus` (0.16 s) only for a
   narrow Pareto scan.  Production remains the existing `balanced` 0.20 s
   cadence.
8. Selected `RelWithDebInfo` for the benchmark/build handoff.  It keeps native
   optimization and symbols; no assert, integrity or rollback logic was
   removed.

## Reverted experiment

An in-place full-Hessian factor accumulation experiment increased graph
linearization P50 from 7.764 to 8.406 ms, solver median from 52.741 to
59.869 ms, and lowered RTF from about 0.436 to 0.413.  It was fully reverted.

## Isolated effects

The exact marginal transform lowered graph linearization P50 from 8.406 to
5.851 ms.  The snapshot change lowered its own P50 from 3.943 to 0.303 ms
(92.3%).  Vectorized reprojection lowered visual-factor P50 from 1.822 to
0.475 ms (73.9%).  Map batching lowered LiDAR integration P50 from 12.511 to
6.678 ms and RGB-D integration from 70.757 to 26.023 ms in its controlled A/B;
structured publication then lowered publication P50 from 15.011 to 7.894 ms.

All algorithmic changes transfer to real hardware.  No Gazebo physics, sensor
rate, timestamp, LiDAR geometry sample cap, D_V threshold, association gate,
integrity threshold or rollback policy was changed.
