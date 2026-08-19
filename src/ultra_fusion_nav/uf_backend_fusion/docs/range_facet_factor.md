# RangeFacet factor

This is an experimental, disabled geometry module. It evaluates one range
measurement as the intersection of a sensor ray and a local plane facet:

    p_s = p_b + R_wb t_bs
    u_w = R_wb R_bs u_s
    r_hat = -(n^T p_s + d) / (n^T u_w)
    r = r_hat - r_measured

The evaluator gates range limits, facet support and fit quality, dynamic
flags, timestamp skew, ray/plane parallelism, positive intersection and
intersection support. It propagates plane and pose covariance into the
measurement variance and applies a Mahalanobis gate followed by a Huber
weight.

The sliding-window integration is behind `range_facet_enabled` and is off in
the frozen baseline. When enabled, MTF-01P optical flow and range share one
packet and correlated timing, so the backend stores one composite MTF factor:
two optical-flow rows and, only when this evaluator accepts the geometry, one
RangeFacet row. One packet ID is consumed once. The range source is never
added again as a height factor or as a second map factor.

A facet should come from a stable, time-associated local submap, D435i depth,
or a static map, and must be rejected when the source is geometrically or
temporally uncertain. Facets produced by the same current scan must not be
used as independent evidence without accounting for their correlation.

The scalar range row is not automatically a Z observation. Its translational
information is obtained from the translation block of J^T W J. Directional
scheduling should project that information onto map X/Y/Z and only hand
weight to RangeFacet on axes where it contributes independent information.
This follows the existing axis-specific LiDAR handoff boundary: a weak LiDAR
axis is reduced, while strong LiDAR axes remain unchanged.

The geometry follows the arbitrary-structure range extension in
Range-Visual-Inertial Odometry and the structure-invariant extension. The
implementation is only a local building block; it does not claim that the
current simulated range source is calibrated for real MTF-01P noise.
