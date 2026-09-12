# HXY-BASELINE-001 ： LiDAR水平退化审计
# #范围和证据
本次审核比较了稳定基线`c7c1adcd92a7fdd3b5b38aa47e48a10ea3552981`
有PR17参考文献`4587e479d5f02dbaaaff048c266fd873d124d109`。它是只读算法审核：无融合行为、Z处理、状态机、或重新定位逻辑已更改。
初步证据(%)
-两个精确提交时的稳定和PR17源代码；-现有PR17报告`docs/experiments/lidar_degeneracy_20260825.md`;-`logs/lidar_deg_visual_anchor_direct_straight_1ms_final_20260825`;-`logs/paper_eq19_long_tunnel_8m_20260825_rerun`;-现有的重播和运行时仪表。未运行新的模拟。
日志位于原始存储库工作树中，不是提交数据。因此，应从冷冻袋中复制基于它们的结论在成为新的基线声明之前。
# #执行结论
稳定后端检测到方向性弱点，但一般不应用检测到传输到激光雷达信息的任意弱本征空间优化器。正常路径使原始点对平面对应关系重新线性化，将完整的LiDAR因子乘以一个调度标量。现有轴handoff可以缩放映射X/Y/Z平移列，但仅当替代传感器提供足够的同轴信息。在PR17隧道运行中，它没有：报告的量表保持不变`[1,1,1]`切换计数保持为零，尽管明显弱的Y轴。
因此，大故障不是计算特征向量的故障。它是定向证据和因子图之间的控制路径间隙，复合通过预测门配置和固定滞后历史记录。一次坏的LiDAR更新被录取，其信息可以一起被吸收到边缘之前使用IMU和辅助因素。此前不再保留源所有权，因此旧的LiDAR信息以后不能完全通过模态或任意亚空间
PR17增加了纸质Eq.19分数，二进制Eq.15录取，可重复隧道几何/任务控制，以及更好的实验布线。它没有实现任意子空间信息缩放、历史激光雷达重新加权或公平单变量重放比较。其严格的二进制路径还可以删除整个激光雷达因子，而不是保留强大的方向。
# # 1.Hessian到优化器的数据流
完整的稳定数据路径是：
1.已修补的FAST-LIO发布`NativeLidarFactor`开   `/fast_lio/native_lidar_factor`.数据包包含完整的正态方程，线性化姿势，剩余能量，方差，对应计数，原始匹配的点/平面、外在、扫描边界、序列和重置epoch。2.`native_factor_from_message()`验证形状、有限性、帧、状态order, correspondence payload, and positive-semidefiniteness.它提取6自由度姿势法线，将FAST-LIO右-SO (3)旋转坐标转换为后端姿势坐标。变换后的法线和本机右侧正常被保留。3.对于每次接受的扫描，`lidar_pose_observability()`range-normalizes 6x6Hessian和报告秩、条件和最弱的6自由度特征向量。   `lidar_vertical_observability()`形成翻译Schur补语   `S_t = H_tt - H_tr pinv(H_rr) H_rt`，然后报告其三个特征值，最弱的3-D平移特征向量和轴投影。名称，此函数分析所有XYZ平移方向。4.`lidar_reliability_layers()`导出健康、预测一致性和XYZ支持诊断。`axis_observability_latch()`增加了轴滞后。5.外部可靠性监视器独立产生一个标量激光雷达降级分数。调度程序将其转换为`factor_enabled`第一部分   `reliability_weight`，和一个`covariance_inflation`.稳定的锚固保护当分数证据过时或较弱时，可能会保留LiDAR。6.后端首先插入相同时间的GNSS ，然后评估可选的轴切换。Handoff仅生成三个地图轴比例，并且仅当配置的替代方案信息源可以采用一个轴。它不是一个本征空间投影仪。7.在法向流形路径上，原始点平面对应被重新线性化在每次非线性迭代中。决策标量对整个因子进行加权。如果没有原始对应关系，则插入6x6压缩法线；它仍然由一个标量加权。线性模式使用坐标转换凝结正常以相同的方式。8.解算器结合了LiDAR、IMU、GNSS、光流、RGB-D/VISUAL和PRIORS活动的固定滞后窗口。当旧状态离开窗口时，所有因素触摸它被Schur补充成一个源不可知的`marginal_prior`。
因此，系统具有必要的电流帧几何形状，但默认情况下优化器合约仍然是全因子标量加权加上可选的XYZ轴列缩放。
# # 2.为什么检测没有变成定向减权
有四个不同的原因。
首先，秩检测器使用相对`1e-3`cutoff.A方向可以是很多比同行弱，而6-DoF和翻译队伍仍保持满员。PR17直接直线运行以归一化平移特征值结束`[0.0972,0.8465,1]`：相对较弱，但没有排名缺陷threshold.Its`native_lidar_directionally_degenerate`计数器因此停留零，即使轴诊断标记为Y弱。
其次，调度程序输出是标量。高降级要么膨胀/禁用整个因子，或者在稳定的锚保护下，可以保留它。操作适用`V diag(s_i) V^T`在测量的弱基础中。
第三，唯一稳定的定向执行器是轴切换。它仅限于映射轴并取决于替代信息。在直线日志中：
-最弱方向：`[0.0076,0.9997,-0.0229]`;-轴相对支撑：`[0.915,0.102,1.000]`;-轴信息刻度：`[1,1,1]`;-切换帧：`0`;-最终诊断时的替代信息：`[0,0,0]`。
测量了弱方向，但未删除优化器信息。
第四，浓缩形式和边缘形式失去了原始的残差行身份需要用于以后的选择性权重调整。压缩电流因子保留了6x6正常，可以在插入前进行光谱修改，但已经形成边际先验混合模式，事后无法精确分解。
# # 3.预测拒绝/恢复和协助互动
# # #预测门
预测创新将激光雷达姿势与IMU/窗口运动进行比较reference.If enabled and position or yaw exceeds its gate, the current LiDAR因子被拒绝。在配置的连续拒绝次数之后，可用满秩原始几何形状可以在恢复地板重新进入（默认重量`0.2`,通货膨胀`5`） ，而不是全强度。如果出现以下情况，则还会拒绝恢复调度程序已禁用LiDAR。
这种机制可以阻止不良电流因素，但它有两个反馈危险：
-预测来自已受历史激光雷达影响的状态；-反复拒绝留下传播/帮助进行估计，而弱或缺少辅助集会增加预测误差并阻止恢复。
关键的PR17日志没有行使这种保护。无论是成功的2米直接运行和发散8米重运行报告预测门禁用，零闸门剔除，零回收和零回收地板因子。8米重新运行以职位创新结束，关于`8246 m`，但仍在继续处理激光雷达直到事务完整性被拒绝/回滚180次更新。较小的直接运行也结束于`3.42 m`创新，而其配置的1米大门被禁用。
GNSS
GNSS是这些运行中最强的绝对水平漂移抑制器。在LiDAR之前插入，在NIS中使用预测状态协方差，并分成水平和垂直准入。有效的连续修复保留了非零的鲁棒性地板，即使经过大型创新。在2米直跑中，形成了154/157次尝试具有近单位有效信息和非常小的XY NIS的GNSS因子；这是与小的XY RMSE一致（`0.0402 m`） ，尽管LiDAR Y几何形状较弱。
然而，如果时间戳选择、调度程序有效性、稳健的扩展性，或者求解器事务拒绝其更正，或者绝对因子被累积/边缘化的相对information.Counts单独是不够的；预计的信息和接受状态校正是必需的。
# # # RGB-D/视觉直接因子
RGB-D直接因素可以限制视觉纹理区域的姿势，并且与相邻的LiDAR键控窗口状态相关联。PR17故意添加相机专用纹理而不改变碰撞/LiDAR几何形状，这是一个有用的模式分离。2 m运行形成并解算器接受所有52次尝试直接因素和保持准确。这证明视觉加GNSS可以抑制弱激光雷达方向，不证明激光雷达退化得到纠正。8米重测接受了142个视觉因子，但仍有分歧，表明该因子仅存在不能在漂移中建立足够的信息子空间或在图形调节恶化后成功校正。
# # #光流
流量影响水平位移/速度，而不是绝对全局位置锚。它可以减缓短期漂移时，质量，旋转，速度，范围和调度门承认它，但它自己的集成会累积错误。2米直跑在316次尝试中形成了177次；发散的8米重跑形成了零。这种差异是一个主要的混淆因素，必须在重播中加以控制。
### Historical window

Within the active window, individual LiDAR factors still retain correspondence
or normal-equation identity and can in principle be rebuilt before solving.After marginalization, their contribution is inseparably mixed with IMU, GNSS,
flow, vision and old priors.A later detector cannot retroactively downweight
only historical LiDAR without retaining source-separated marginal components or
rebuilding from a longer factor history.
## 4.Weak direction versus final Y drift

The vector `[0.992,-0.107,-0.072]` and a final Y drift are not inherently
contradictory, but that vector alone does not prove the cause of Y drift.
If the vector and final error are expressed in the same fixed map frame at the
same time, it is mostly X-directed; a pure Y error projects onto it with magnitude
only about `0.107 |e_y|`.In that narrow interpretation, calling the final Y
error a direct consequence of this one snapshot would be unsupported.
The broader trajectory can still produce Y drift because:

- eigenvectors are instantaneous and can rotate/sign-flip with scan geometry;
- map, body, route and truth-aligned frames are different, and yaw error rotates
  horizontal error between X and Y;
- drift is the time integral of biased increments, not the final Hessian alone;
- old weak directions have already entered the marginal prior;
- GNSS/vision/flow information and admission change over time.
The existing PR17 direct-run final snapshot actually reports a nearly pure Y
weak direction `[0.0076,0.9997,-0.0229]`, while the divergent 8 m rerun ends at
`[-0.0020,0.99998,-0.0063]`.The cited X-dominant vector is therefore likely a
different scan, phase, run or frame.A time series with explicit frame labels is
required before reconciling it with final Y error.
## 5.What PR17 solved and what remains

PR17 solved or materially improved:

- an opt-in Eq.19-only LiDAR score;
- opt-in binary Eq.15 admission with stable anchor/stale-score overrides removed;
- explicit experimental configuration plumbing and tests;
- a camera-textured but LiDAR-geometry-preserving tunnel;
- a straight mission and landing-aware observers;
- useful successful 2 m and ordinary-indoor feasibility evidence;
- clearer separation of received packets from solver admission in its report.
PR17 did not solve:

- arbitrary weak-eigenspace scaling of LiDAR information;
- temporal smoothing/tracking of the weak basis;
- replay-time reweighting of historical LiDAR already in marginal priors;
- source-projected information accounting for GNSS, RGB-D and flow;
- enabled prediction gating in the reported critical runs;
- a one-variable A/B comparison on identical sensor messages and timestamps;
- robust long-distance behavior: the 8 m rerun diverged catastrophically, and
  the selected 2 m result still failed its sustained-error-duration gate;
- proof that visual/flow factors constrain the same weak subspace rather than
  merely being present in the graph.
Binary Eq.15 admission is useful as a diagnostic extreme, but it is not the
desired subspace solution: it discards strong LiDAR directions together with the
weak one and can reduce graph observability.
## 6.Is the current history sufficient for arbitrary-subspace reweighting?
The answer is conditional:

- **Current active raw-correspondence factors: yes.** They retain points, plane
  normals/points, extrinsics and scan linearization, so residual Jacobians can be
  relinearized and transformed by a 3-D projector.- **Current active condensed factors: partly.** The 6x6 Hessian and gradient are
  enough for a mathematically consistent local spectral/Schur modification, but
  not for changing individual robust residual weights or correspondences.- **Factors absorbed into `marginal_prior`: no.** Source and subspace ownership
  are lost.Exact post-hoc LiDAR-only reweighting is impossible.- **Existing saved runtime JSON: no.** It records mostly latest values and
  summaries, not every scan's full Hessian/gradient, basis, factor decision and
  marginalization lineage.- **A frozen rosbag containing complete `NativeLidarFactor`: potentially yes for
  replay from the beginning.** Rebuild every window under A/B/C; do not attempt
  to mutate a graph midway and call it fair.
## 7.Fair A/B/C replay contract

Use one immutable bag, identical start state, deterministic ordering, solver
settings, window size, iteration budget and all non-LiDAR parameters.Disable
truth feedback into the estimator.Change exactly one LiDAR policy:

- **A:** stable `hybrid/adaptive`, current scalar/axis-handoff behavior;
- **B:** PR17 `paper_eq19/paper_eq15`, binary whole-factor control;
- **C:** proposed arbitrary-subspace scaling, preserving strong eigen-directions.
For every LiDAR scan record:

- scan/epoch/sequence and begin/end/factor timestamps;
- received, parsed, selected, correspondence-valid, attempted, graph-added,
  enabled, solver-accepted, rejected and rollback outcome, with reason;
- matched/candidate points, variance and residual energy;
- full 6x6 Hessian and gradient in a named coordinate convention;
- 3x3 translation Schur information, ordered eigenvalues/eigenvectors, rank,
  condition, basis frame, sign convention and inter-frame basis angle;
- scalar scheduler score, weight, inflation, anchor override, prediction-gate
  decision, consecutive rejects, recovery-floor decision and effective weight;
- actual subspace projector/scales applied to H and g, plus information trace
  before/after;
- LiDAR prediction position vector (not only norm), yaw innovation and thresholds;
- per-factor residual/NIS before solve, after solve and robust loss scale.
For each solver transaction record:

- state before prediction, predicted state, initial graph state, optimized state,
  and correction vectors in map and weak-basis coordinates;
- factor counts and effective information matrices from LiDAR, GNSS, flow,
  RGB-D/visual and IMU, projected onto the same LiDAR eigenbasis;
- GNSS XYZ residual, XY/Z NIS, robust scales, admission and factor formation;
- flow displacement/covariance, quality/range/rotation gates and formation reason;
- RGB-D/visual track count, rank/condition, residual, information projection,
  association stamps and solver acceptance;
- total information rank/condition, cost before/after, LM rejects, integrity
  result, rollback, marginal covariance and output covariance;
- factors entering marginalization and source-separated information contributed
  to the new prior (diagnostic accounting even if the production prior remains
  combined).
For each run report:

- 3D, XY and Z RMSE; P95; maximum and endpoint error; per-axis RMSE and endpoint;
- error projected onto instantaneous and route-aligned weak/strong directions;
- received/selected/attempted/formed/enabled/solver-accepted/rejected counts for
  LiDAR, GNSS, flow, RGB-D/visual and IMU;
- prediction reject/recovery streak distributions and time spent per LiDAR mode;
- first divergence time, weak-direction history, aiding availability at that
  time, solver condition, rollback intervals and marginalization lag;
- timing/RTF only as a secondary check that A/B/C consumed the same event stream.
Do not compare the existing successful 2 m direct run with the divergent 8 m run
as A/B: route length, flow formation, sensor timing and graph history differ.
## Recommended next step

Freeze one long-tunnel bag that contains complete Native LiDAR factors and all
four aiding streams, then first add diagnostic-only per-scan JSONL capture of the
quantities above.Replay A and B unchanged to prove deterministic equivalence and
locate the first divergence transaction.Only after that evidence is complete,
implement C as a current-window LiDAR subspace transform from the full 3x3 Schur
eigenbasis.Restart each replay from the beginning so marginal priors are formed
under the selected policy; do not change Z, state-machine, or relocalization
behavior in this experiment.