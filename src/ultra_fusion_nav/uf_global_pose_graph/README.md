# Offline global pose graph

`uf_global_pose_graph` is an offline-only SE(3) optimizer. It consumes immutable
keyframe poses plus loop edges already accepted by the strict
`uf_relocalization/offline_loop_smoke` geometric frontend. It never subscribes
to, publishes to, or modifies the online fixed-lag fusion backend.

The first node is fixed. Sequential odometry edges preserve local motion, while
loop edges use DCS robust weighting and post-fit quarantine. Before optimization,
nearby edges with similar relative transforms are clustered so exact correlated
constraints contribute one representative. Distinct constraints from the same
continuous revisit episode are retained for trajectory coverage but normalized
by `1/sqrt(N)`, so their total information equals one independent observation.
Every decision remains in `loop_edge_audit.json`.

For reproducible offline retrieval, `offline_loop_smoke` accepts
`--descriptor-seed`. PCL 1.12 otherwise seeds ESF sampling from wall-clock
seconds inside the shared feature library. The fixed seed symbol is linked only
into the offline executable; the production relocalization node is unchanged.

Corrected poses are emitted as a new revision. Original scan poses, dense
trajectory, raw scan archive, and frontend output remain untouched. The existing
`uf_map_maintenance` builder can then rebuild a cleaned map from the corrected
scan and per-point deskew trajectory revisions.

```bash
ros2 run uf_global_pose_graph offline_global_pose_graph \
  --keyframes /data/keyframes/keyframes.csv \
  --loop-frontend /data/loop_frontend_v2.json \
  --session /data/session \
  --output /data/global_pose_graph/loop-0001 \
  --revision loop-0001 \
  --config install/uf_global_pose_graph/share/uf_global_pose_graph/config/global_pose_graph.yaml \
  --map-output /data/maps/corrected \
  --repeat-map-output /data/maps/corrected_repeat \
  --original-map /data/maps/original/cleaned_map.pcd
```

Generate the geometrically verified edge file first:

```bash
ros2 run uf_relocalization offline_loop_smoke \
  --metadata /data/keyframes/keyframes.csv \
  --output /data/loop_frontend_v2.json \
  --minimum-separation-s 20 \
  --maximum-candidates 5 \
  --exclude-recent 3 \
  --voxel-size-m 0.15 \
  --descriptor-seed 1731
```

The tool reports internal loop and map consistency only. Without independent
truth it does not report ATE or claim an absolute localization improvement.
