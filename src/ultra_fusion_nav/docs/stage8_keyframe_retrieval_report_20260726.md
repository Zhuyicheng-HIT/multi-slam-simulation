# Stage 8 Static Keyframe Retrieval Core Report

Date: 2026-07-26

## Scope

This milestone connects static-keyframe admission, coarse place retrieval,
and the existing PCL registration core. It remains an offline C++ library and
test pipeline. It does not publish a pose, reset the estimator, modify
ExternalNav, or claim online relocalization.

## Static keyframe admission

`StaticKeyframeDatabase` accepts a keyframe only when all of the following
conditions hold:

- the scheduler still enables the LiDAR factor;
- map quality is at least 0.60;
- feature repeatability is at least 0.70;
- dynamic ratio is at most 0.15;
- LiDAR degradation is at most 0.75;
- the cloud, timestamp, pose, and descriptor are finite and valid;
- translation or rotation from the last accepted keyframe exceeds the
  configured spacing threshold.

Accepted clouds are deep-copied so later publisher-buffer changes cannot
mutate the static map. Descriptor dimension is fixed by the first keyframe.
Storage is bounded and evicts the oldest frame while keeping monotonically
increasing keyframe IDs.

## Coarse retrieval

The database stores normalized descriptors and ranks candidates by cosine
distance. Queries can exclude a configurable number of recent keyframes before
returning the best bounded candidate set.

PCL's maintained ESF implementation supplies the first runnable 640-dimensional
global shape descriptor. ESF is a baseline provider for the database interface;
it is not presented as Scan Context or as an Ultra-Fusion source component.
The descriptor core filters non-finite points, requires at least 10 finite
points, and rejects malformed output.

## Registration handoff

Candidate retrieval returns only keyframe IDs and descriptor distances. The
caller obtains the stored static cloud and passes it to the separately tested
PCL NDT/ICP wrappers. A pipeline test performs:

```text
quality-gated static keyframe -> descriptor query -> candidate cloud
    -> ICP geometric verification -> known rigid-transform recovery
```

The descriptor never directly authorizes a pose correction.

## Verification

`uf_relocalization` now has four passing CTest targets containing nine GTest
cases. `colcon test-result` reports 13 total test records when the four target
wrappers are included:

- 3 ICP/NDT registration tests;
- 3 keyframe admission, bounded-storage, and candidate-ranking tests;
- 2 PCL ESF descriptor tests;
- 1 retrieval-to-ICP pipeline test;

The keyframe tests verify scheduler and quality rejection, pose spacing,
cloud deep-copy ownership, bounded eviction, descriptor-dimension enforcement,
recent-frame exclusion, and deterministic ranking. The ESF test verifies that
a rigidly transformed version of the same asymmetric cloud ranks ahead of a
planar distractor.

## Current boundary

- No ROS node or rosbag keyframe builder exists yet.
- No keyframe database serialization or reload exists yet.
- ESF candidate recall and false-match rate have not been measured on a real
  repeated-place sequence.
- Scan Context has not been integrated.
- Registration fitness, cross-sensor consistency, and scheduler recovery gates
  are not yet combined into a final acceptance decision.
- No estimator or FCU reset counter is emitted.

## Next gate

1. Build an offline rosbag keyframe extractor using `/lidar/static_cloud`,
   `/lio/odom`, `/lio/diagnostics`, LiDAR reliability, and scheduler state.
2. Measure keyframe admission rate, descriptor recall, and wrong-candidate rate
   on a repeat traversal before adding an online node.
3. Run NDT for coarse verification and ICP for refinement, then reject by
   convergence, fitness, transform jump, and cross-sensor consistency.
4. Add a versioned keyframe-map format only after the admission and retrieval
   metrics are stable.
