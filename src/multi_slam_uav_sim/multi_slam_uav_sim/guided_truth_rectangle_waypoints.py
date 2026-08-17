"""Rectangle diagnostic controlled by Gazebo truth with SLAM observer-only."""

import rclpy

from .guided_rectangle_waypoints import GuidedRectangleWaypoints
from .guided_s_curve_waypoints import GuidedSCurveWaypoints


class GuidedTruthRectangleWaypoints(GuidedSCurveWaypoints):
    """Reuse the proven truth-to-FCU adapter for the short rectangle route."""

    def __init__(self):
        super().__init__(
            node_name="guided_truth_rectangle_waypoints",
            enforce_figure8_constraints=False,
        )
        if self.route_feedback_source != "gazebo_truth":
            raise ValueError(
                "guided_truth_rectangle_waypoints requires "
                "route_feedback_source=gazebo_truth"
            )

    def activate_route_control(self):
        super().activate_route_control()
        self.home_x, self.home_y, self.home_z = self.route_origin_feedback
        self.home_yaw = self.route_origin_feedback_yaw

    def run(self):
        return GuidedRectangleWaypoints.run(self)


def main(args=None):
    rclpy.init(args=args)
    node = GuidedTruthRectangleWaypoints()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
