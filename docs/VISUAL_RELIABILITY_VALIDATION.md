# Visual reliability 验证

升级后的 vision degradation score 使用真实 track 证据：

`D_core = 0.30 D_count + 0.25 D_grid + 0.25 D_reprojection + 0.20 D_depth`

`D_V = 0.85 D_core + 0.15 (1-r_KLT)`

Feature support 是可用 inlier 数量，spatial distribution 是 8x8 grid 中被占用的 cell，reprojection 使用 PnP/RANSAC inlier residual，KLT 使用 forward-backward consistency。Transport、depth validity、blur 和 brightness 仍是补充性的工程证据。PnP 仅作为 validity gate；其 pose 永远不会插入 backend。

Unit test 覆盖 count/grid 行为、确定性的 KLT/PnP tracking 以及无效几何。最终 headless run 同时观察到 `/vision/feature_tracks` 和 `/reliability/vision_score`，确认真实 frontend 与 reliability node 已激活。在 native backend startup timeout 前未能证明有效的 live score 和 factor acceptance，因此 flight-level D_V 行为标记为 PARTIAL。
