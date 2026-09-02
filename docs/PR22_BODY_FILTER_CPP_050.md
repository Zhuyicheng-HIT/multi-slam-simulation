# PR22-BODY-FILTER-CPP-050

## Scope and baseline

- Base: PR21 `integration/runtime-consolidated-v1`
- Base commit: `234fc3c13568c63b8c8f35e718860a67df8bebef`
- Development branch: `codex/pr22-body-filter-cpp-050`
- Scope: replace only the production Python `pointcloud_body_filter` process
  with a C++ ROS 2 node. The Python implementation remains available for A/B.

No fusion, Dynamic, HXY, relocalization, sensor model, topic, QoS, or body-mask
geometry changes are included. Raw `/livox/lidar` remains available unchanged.

## Implementation

The C++ core performs one linear pass over the cloud and pre-reserves the
maximum output size. It reads `x/y/z` through the declared field offsets,
datatypes, and endian flag, then copies each retained point's entire original
record. It does not deepcopy the input cloud and does not create a Python bytes
object per point.

Production launch keeps the node name and the existing parameter-file node key
`pointcloud_body_filter`; only package/executable resolution changes to
`uf_pointcloud_body_filter_cpp/pointcloud_body_filter_cpp`.

## Frozen replay

The controlled input is the immutable `[15 s, 45 s)` slice from the existing
MID360+IMU replay. It contains exactly 300 `PointCloud2` messages and 6000 IMU
messages over 29.995 seconds. The derived bag is excluded from Git.

- Derived SQLite SHA256:
  `ff6c08456f2d86eb80bc25d6f52f91f1c7cf93ff7e3dfc225eeb1e1831f6ce73`
- Each variant was run three times, interleaved, with CycloneDDS and a unique
  ROS domain.
- Values below are the median of the three per-run measurements.

| Metric | Python PR21 | C++ PR22 | Change |
|---|---:|---:|---:|
| Process CPU P50 | 78.20% | 3.93% | -94.97% |
| Process CPU mean | 76.69% | 3.30% | -95.70% |
| Process CPU P95 | 83.63% | 5.92% | -92.92% |
| Callback P50 | 78.680 ms | 1.107 ms | -98.59% |
| Callback P95 | 98.525 ms | 1.563 ms | -98.41% |
| LiDAR transport age P50 | 179.715 ms | 103.079 ms | -42.64% |
| LiDAR transport age P95 | 205.903 ms | 109.171 ms | -46.98% |
| Peak RSS | 56.02 MiB | 58.05 MiB | +2.03 MiB |

All six runs processed 300 callbacks and produced 300 outputs. There were zero
missing or additional output timestamps. DDS graph size was unchanged (body
filter: two publishers and two subscriptions including `/clock`; profiled graph:
six publishers and five subscriptions).

The captured real bag contains no points inside the default self-body bounds,
so its observed body-removal ratio is 0% for both variants. A deterministic ROS
integration test exercises a non-zero 50% removal case, while core tests cover
range rejection, non-finite points, extrinsics, big-endian layouts, organized
cloud flattening, and complete retained-record preservation.

## Equivalence and verification

On all 300 frozen LiDAR frames, Python and C++ outputs had identical timestamps,
headers, fields, dimensions, endian metadata, point/row steps, dense flags, data
lengths, full data SHA256 values, and body-removal ratios: zero mismatches and
zero duplicates.

- Full workspace build: 19 packages PASS.
- Full `colcon test`: PASS; structured result index reports 112 tests, 0 errors,
  0 failures, 0 skipped. Python package unittest suites also exited cleanly.
- New package: 6 core GTests + 1 live ROS 2 node contract test PASS.
- `uf_sensor_pipeline`: 47 tests PASS, including production launch resolution.
- `git diff --check`: PASS.

The approximately 100 ms floor in C++ transport age is consistent with the
recorded scan/header timing contract. The Python path adds a backlog above that
floor; the C++ path removes nearly all of it without changing sensor semantics.
