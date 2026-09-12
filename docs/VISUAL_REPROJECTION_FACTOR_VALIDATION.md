# Visual reprojection factor 验证

## Contract

`VisualFeatureTracks` 携带精确的 current/previous stamp 及每条 track 的 ID、normalized 与 pixel 坐标、metric depth/inverse depth、age、KLT forward-backward error、geometric-inlier 标志和 reprojection error。backend 应用配置的 camera time offset 后，将两个 stamp 关联到相邻的 window state。Paper mode 只启用该 local visual factor；RTAB odometry 仅在明确的 legacy A/B mode 中可用。

## 数学检查

- 两个位姿的 analytic right-local SE(3) Jacobian 在确定性测试中与中心流形有限差分一致。
- 在优化前拒绝非有限观测、非正 depth/inverse depth、投影到 camera 后方以及无效 covariance。
- 2.5-sigma Huber loss 限制单个 normalized-image residual。
- Information 乘以现有 scheduler 提供的 `reliability_weight/covariance_inflation`；不存在第二个 scheduler。
- Admission diagnostics 区分 timestamp/window 不匹配、depth 有效的 KLT/geometric track 不足、scheduler disable 和 geometric failure。

## 结果

四项 factor 测试全部通过，包括有限差分 Jacobian、无效输入拒绝和位姿校正。在三 seed 合成 A/B harness 中，paper reprojection 的平均平移 RMSE 为 0.002061 m、旋转 RMSE 为 0.000561 rad；four-source factor set 为 0.024529 m/0.019108 rad；legacy RTAB-style relative factor 为 0.010783 m/0.005669 rad。这些是因子级确定性回归结果，不代表飞行或真实世界精度。

最终 headless 尝试中，live frontend 发布了 `/vision/feature_tracks`，但 `/fusion/unified/odom` 未达到首次 committed state。因此，live visual-factor acceptance 仅为 PARTIAL，不能从 topic 存在推断已接受。
