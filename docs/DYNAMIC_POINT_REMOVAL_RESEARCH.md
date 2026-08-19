# Dynamic point removal research and first-layer decision

Review date: 2026-08-19

## Problem boundary

The current `TemporalVoxelFilter` operates after registered clouds and only
protects the project-side local map. It does not prevent a dynamic return from
entering FAST-LIO point-to-plane matching, FAST-LIO's ikd-tree, a
NativeLidarFactor, the shared map, or a relocalization map. The first production
filter must therefore eventually sit before FAST-LIO while retaining a separate
long-term map-refinement path.

This review distinguishes three claims that are often mixed together:

1. online scan segmentation before odometry;
2. online static-map construction using an already estimated trajectory;
3. offline cleaning after the full trajectory/map is available.

Only the first can directly protect FAST-LIO matching. Algorithms in the second
category still have a trajectory dependency that must be broken with causal IMU
prediction, delayed committed states, or a carefully designed two-pass contract.

## High-value primary sources and repositories

### FreeDOM

- Paper: <https://arxiv.org/abs/2504.11073>
- Official code: <https://github.com/LC-Robotics/FreeDOM>
- License: MIT repository license; the ROS package metadata still says `TODO`.
- Public repository snapshot reviewed: 4 commits, latest 2025-04-27.

FreeDOM is class-agnostic. It maintains a coarse FreeSpace and finer StaticSpace,
requires repeated and spatially conservative free observations, enhances rays in
missing-return regions, grows dynamic labels from free-space contradictions, and
uses later free-space evidence to refine the accumulated map. The paper reports
both Livox Avia and MID360 experiments and formulates pose as a general `SE(3)`
transform rather than a planar vehicle state.

The official code is not directly reusable here: it is ROS1/catkin, subscribes
to `sensor_msgs/PointCloud2`, requests an exact TF pose at the cloud stamp, does
not consume Livox `CustomMsg` per-point offsets, has no ROS2 Humble CI, and ships
only a small research-oriented history. Its angular depth-image enhancement also
needs a measured FoV mask/scanning-pattern contract; applying a spinning-LiDAR
range-image assumption to MID360's non-repetitive scan would invent free space.

### DUFOMap

- Paper: <https://arxiv.org/abs/2403.01449>
- Official code: <https://github.com/KTH-RPL/dufomap>
- Benchmark: <https://github.com/KTH-RPL/DynamicMap_Benchmark>
- License: BSD-3-Clause.
- Public repository snapshot reviewed: 103 commits, latest 2025-08-31.

DUFOMap is class-agnostic and marks a voxel as void only when ray casting plus a
3-D neighborhood shows it was fully observed empty. Its paper explicitly makes
no assumption about point-cloud structure, demonstrates non-repetitive MID360
data and large height changes, and provides both online scan queries and offline
map cleaning. Pose and sensor-noise margins are explicit.

This is the strongest reusable geometric core. It is plain C++20 and more mature
than FreeDOM's release, but the repository is a dataset executable around
UFOMap/TBB/LZ4 rather than a ROS2 component. It still consumes pose/cloud pairs,
and its persistent `void` decision needs a recovery policy for drift and
calibration changes before it can protect a safety-critical online frontend.

### Dynablox

- Paper: <https://arxiv.org/abs/2304.10049>
- Official code: <https://github.com/ethz-asl/dynablox>
- License: BSD-3-Clause.
- Public repository snapshot reviewed: latest 2025-03-09.

Dynablox is an online, class-agnostic TSDF/ever-free approach with spatial and
temporal clustering. It is a credible online comparator and its algorithm is
general 3-D rather than ground-bin based. The public stack is nevertheless ROS1
Noetic/catkin on Ubuntu 20.04 with a sizable Voxblox/Kindr dependency graph; the
maintainers explicitly do not guarantee other versions. NVIDIA's nvblox contains
a GPU-oriented descendant, which is not a drop-in CPU ROS2 Humble solution for
this project.

### ERASOR and ERASOR2

- ERASOR official code: <https://github.com/LimHyungTae/ERASOR>
- ERASOR2 official code: <https://github.com/url-kaist/ERASOR2>
- ERASOR2 paper: <https://roboticsproceedings.org/rss19/p067.pdf>
- ERASOR2 license: GPL-3.0.

ERASOR is an offline map cleaner based on egocentric pseudo-occupancy ratios and
ground reversion. Its height bins, ground assumptions, and per-sensor tuning are
poor fits for a banking/pitching UAV. ERASOR2 improves offline quality with
instance awareness, Patchwork ground extraction, and HDBSCAN. Its current code
is ROS-free and builds on Ubuntu 22.04, but it requires the complete pose/scan
sequence and preprocessing; it cannot clean a scan before FAST-LIO. GPL-3.0 also
precludes copying its implementation into this Apache-2.0 package without a
project-wide licensing decision.

ERASOR2 remains useful as an offline evaluation/map-refinement comparator, not
as the first layer.

### DynamicMap Benchmark and other methods

The KTH benchmark currently groups:

- online/no-prior: DUFOMap, OctoMap with ground filtering, Dynablox, OctoMap;
- learning-based: DeFlow;
- offline/prior-map: BeautyMap, ERASOR, Removert.

BeautyMap and ERASOR use ground-oriented abstractions. Removert repeatedly
projects a completed map into range images. They can improve final maps but do
not meet the pre-FAST-LIO requirement. Deep MOS families such as MapMOS,
MF-MOS, MambaMOS, and 2DPASS-MOS provide useful research comparisons, but add
training weights/GPU or sparse-convolution dependencies and are dominated by
spinning-automotive data. HeLiMOS adds solid-state Livox Avia labels and is a
valuable future cross-sensor dataset, although its data license is
CC BY-NC-SA and it is not a substitute for team MID360/UAV data.

Recent AWV-MOS-LIO work is architecturally relevant because it couples adaptive
visibility windows to LIO, but this audit did not find a maintained official
ROS2 Humble/MID360 release that can be reused. It should be revisited if the
authors publish the full implementation.

### FAST-LIO integrations and ROS2 availability

The public `better_fastlio2` fork embeds SCV-OD-style clustering directly inside
a ROS1 FAST-LIO/Scan-Context/GTSAM monolith and documents a speed/precision
trade-off. That violates this phase's no-core-modification rule and would merge
duplicate backend/relocalization functionality. Other Gitee MID360/FAST-LIO
projects found in the audit either perform manual CloudCompare post-processing
or provide localization plumbing, not a validated online dynamic filter.

No reviewed repository simultaneously provided all of:

- official or well-maintained provenance;
- ROS2 Humble;
- Livox MID360 `CustomMsg` with per-point timing;
- online class-agnostic scan output before FAST-LIO;
- UAV 6DoF evidence;
- a permissive license and isolated component boundary.

## Comparative decision

| Method | Class agnostic | Free/visibility cue | Online scan output | Pose dependency | MID360 / irregular evidence | UAV 6DoF risk | ROS2 Humble reuse | Decision |
|---|---|---|---|---|---|---|---|---|
| FreeDOM principle | yes | conservative free space + ray enhancement | yes | estimated `SE(3)` pose | Avia and MID360 reported | no planar equation, but no UAV flight evidence | no | primary algorithm family |
| FreeDOM repository | yes | full front/back structure | ROS1 only | exact TF at scan stamp | dataset configs, no CustomMsg | circularity unresolved | poor | do not port verbatim |
| DUFOMap | yes | fully observed void voxels | yes, plus offline | pose/cloud pairs | explicit MID360 and no scan-structure assumption | lowest structural risk | medium C++ port | strongest core reference |
| Dynablox | yes | TSDF ever-free | yes | registered clouds | heterogeneous data adaptation exists | general 3-D, dependency cost | poor ROS1 stack | comparator, not first choice |
| ERASOR/2 | mostly geometry/instances | pseudo occupancy / ground | no | complete trajectory/map | HeLiPR/HeLiMOS tooling | strong ground-vehicle assumptions | offline only | backend comparator only |
| TemporalVoxelFilter | yes | persistence only | project-side only | registered world cloud | sensor agnostic | new-FoV false positives | already present | mandatory baseline |
| learned MOS | varies | learned motion/semantics | yes | model dependent | mainly automotive; some Avia | domain-shift and GPU risk | low | later optional benchmark |

## Recommendation

FreeDOM is recommended as the **main architectural family**, but its repository
is not recommended as a direct dependency. The first production candidate should
be a clean ROS2/C++ implementation that combines:

1. FreeDOM's temporal conservative free-space confirmation, unknown state,
   occupied recovery, scan-level label growth, and separate map refinement;
2. DUFOMap's fully-observed spatial neighborhood and explicit pose/noise margins;
3. the existing Livox `CustomMsg` per-point timing and the same causal IMU deskew
   trajectory used by FAST-LIO;
4. bounded queues, fail-open diagnostics, and no truth access.

The first observer implements items 1, 3 (using delayed committed poses only in
observer mode), and 4. A true MID360 angular raycast enhancement is intentionally
not enabled until a measured scan-pattern/FoV mask proves it does not create
false free space. The long-term map-refinement backend remains a separate next
step.

## Proposed production boundary

```text
Raw MID360 CustomMsg
  -> causal IMU deskew / pose prediction shared with FAST-LIO
  -> spatial + temporal conservative free-space observer
  -> validated MID360 ray enhancement (optional, evidence-gated)
  -> scan-level dynamic/unknown segmentation
  -> clean scan gateway
  -> FAST-LIO
  -> NativeLidarFactor -> unified backend

raw/static/dynamic evidence + committed poses
  -> asynchronous map-refinement backend
  -> shared/relocalization static map products
```

The gateway is not activated in this phase. The observer subscribes to the raw
stream in parallel and FAST-LIO continues to receive the original topic.
