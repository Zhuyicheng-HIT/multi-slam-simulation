# Visual reliability validation

The upgraded vision degradation score uses real track evidence:

`D_core = 0.30 D_count + 0.25 D_grid + 0.25 D_reprojection + 0.20 D_depth`

`D_V = 0.85 D_core + 0.15 (1-r_KLT)`

Feature support is the usable inlier count, spatial distribution is occupied
cells in an 8x8 grid, reprojection uses the PnP/RANSAC inlier residual, and KLT
uses forward-backward consistency. Transport, depth validity, blur and
brightness remain supplementary engineering evidence. PnP is a validity gate
only; its pose is never inserted into the backend.

Unit tests cover count/grid behavior, deterministic KLT/PnP tracking and
invalid geometry. The final headless run observed both
`/vision/feature_tracks` and `/reliability/vision_score`, confirming the real
front end and reliability node were active. A valid live score and factor
acceptance could not be demonstrated before the native backend startup timeout,
so flight-level D_V behavior is PARTIAL.
