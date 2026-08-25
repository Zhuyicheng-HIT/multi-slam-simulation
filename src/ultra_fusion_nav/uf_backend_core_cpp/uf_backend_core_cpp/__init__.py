from ._core import (
    imu_preintegrated_cost,
    imu_preintegrated_graph_normal,
    imu_preintegrated_normal,
    lidar_point_plane_cost,
    lidar_point_plane_graph_normal,
    lidar_point_plane_normal,
    marginal_prior_cost,
    marginal_prior_normal,
    rgbd_depth_cost,
    rgbd_depth_normal,
    rgbd_direct_cost,
    rgbd_direct_normal,
    state_plus_batch,
    visual_reprojection_cost,
    visual_reprojection_normal,
)

try:
    from ._core import (
        lidar_point_plane_graph_normal_axis_scaled,
        lidar_point_plane_normal_axis_scaled,
    )
except ImportError:
    # Keep source overlays importable while an older compiled extension is
    # still installed; the fusion package falls back to the Python kernel.
    lidar_point_plane_graph_normal_axis_scaled = None
    lidar_point_plane_normal_axis_scaled = None

__all__ = [
    "imu_preintegrated_cost",
    "imu_preintegrated_graph_normal",
    "imu_preintegrated_normal",
    "lidar_point_plane_cost",
    "lidar_point_plane_graph_normal",
    "lidar_point_plane_graph_normal_axis_scaled",
    "lidar_point_plane_normal",
    "lidar_point_plane_normal_axis_scaled",
    "marginal_prior_cost",
    "marginal_prior_normal",
    "rgbd_depth_cost",
    "rgbd_depth_normal",
    "rgbd_direct_cost",
    "rgbd_direct_normal",
    "state_plus_batch",
    "visual_reprojection_cost",
    "visual_reprojection_normal",
]
