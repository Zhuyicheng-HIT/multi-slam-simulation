import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String

try:
    from mavros_msgs.msg import State
except Exception:
    State = None


class FlightStateBridge(Node):
    """Expose one UAV namespace sourced only from MAVROS/flight-controller data."""

    def __init__(self):
        super().__init__("flight_state_bridge")
        self.declare_parameter("mavros_ns", "/mavros")
        self.declare_parameter("uav_ns", "/uav")
        self.mavros_ns = self.get_parameter("mavros_ns").value.rstrip("/")
        self.uav_ns = self.get_parameter("uav_ns").value.rstrip("/")

        self.state_pub = self.create_publisher(String, f"{self.uav_ns}/state", 10)
        self.local_pose_pub = self.create_publisher(PoseStamped, f"{self.uav_ns}/local_pose", 10)
        self.local_odom_pub = self.create_publisher(Odometry, f"{self.uav_ns}/local_odom", 10)
        self.global_fix_pub = self.create_publisher(NavSatFix, f"{self.uav_ns}/global_fix", 10)
        self.fused_global_fix_pub = self.create_publisher(
            NavSatFix, f"{self.uav_ns}/fused_global_fix", 10
        )
        self.imu_pub = self.create_publisher(Imu, f"{self.uav_ns}/imu", qos_profile_sensor_data)
        self.velocity_pub = self.create_publisher(TwistStamped, f"{self.uav_ns}/velocity", 10)
        self.interface_status_pub = self.create_publisher(String, f"{self.uav_ns}/interface_status", 10)
        mavros_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        if State is not None:
            self.create_subscription(State, f"{self.mavros_ns}/state", self._state_cb, 10)
        else:
            self.get_logger().warning("mavros_msgs is unavailable; /uav/state will not be bridged.")

        self.create_subscription(
            PoseStamped, f"{self.mavros_ns}/local_position/pose", self._local_pose_cb, mavros_sensor_qos)
        self.create_subscription(
            Odometry, f"{self.mavros_ns}/local_position/odom", self._local_odom_cb, mavros_sensor_qos)
        self.create_subscription(
            Odometry, f"{self.mavros_ns}/global_position/local", self._global_local_odom_cb, mavros_sensor_qos)
        self.create_subscription(
            NavSatFix, f"{self.mavros_ns}/global_position/raw/fix",
            self._global_fix_cb, mavros_sensor_qos)
        self.create_subscription(
            NavSatFix, f"{self.mavros_ns}/global_position/global",
            self._fused_global_fix_cb, mavros_sensor_qos)
        self.create_subscription(
            Imu, f"{self.mavros_ns}/imu/data", self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, f"{self.mavros_ns}/local_position/velocity_local", self._velocity_cb, mavros_sensor_qos)

        self.last_seen = {}
        self.timer = self.create_timer(1.0, self._publish_interface_status)
        self.get_logger().info(
            f"Flight state bridge active: {self.mavros_ns}/... -> {self.uav_ns}/..."
        )

    def _touch(self, name):
        self.last_seen[name] = self.get_clock().now()

    def _state_cb(self, msg):
        self._touch("state")
        out = String()
        out.data = (
            f"connected={msg.connected} armed={msg.armed} guided={msg.guided} "
            f"manual_input={msg.manual_input} mode={msg.mode} system_status={msg.system_status}"
        )
        self.state_pub.publish(out)

    def _local_pose_cb(self, msg):
        self._touch("local_pose")
        self.local_pose_pub.publish(msg)

    def _local_odom_cb(self, msg):
        self._touch("local_odom")
        self.local_odom_pub.publish(msg)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._touch("local_pose")
        self.local_pose_pub.publish(pose)

    def _global_local_odom_cb(self, msg):
        # ArduPilot commonly feeds MAVROS global_position/local more reliably than
        # LOCAL_POSITION_NED. This is still FCU/MAVROS data, not Gazebo truth.
        self._touch("global_local_odom")
        if "local_odom" not in self.last_seen:
            self._touch("local_odom")
            self.local_odom_pub.publish(msg)
        if "local_pose" not in self.last_seen:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self._touch("local_pose")
            self.local_pose_pub.publish(pose)

    def _global_fix_cb(self, msg):
        self._touch("global_fix")
        self.global_fix_pub.publish(msg)

    def _fused_global_fix_cb(self, msg):
        self._touch("fused_global_fix")
        self.fused_global_fix_pub.publish(msg)

    def _imu_cb(self, msg):
        self._touch("imu")
        self.imu_pub.publish(msg)

    def _velocity_cb(self, msg):
        self._touch("velocity")
        self.velocity_pub.publish(msg)

    def _publish_interface_status(self):
        now = self.get_clock().now()
        parts = []
        for name in ["state", "local_pose", "local_odom", "global_local_odom", "global_fix",
                     "fused_global_fix", "imu", "velocity"]:
            stamp = self.last_seen.get(name)
            if stamp is None:
                age = "missing"
            else:
                age = f"{(now - stamp).nanoseconds / 1.0e9:.2f}s"
            parts.append(f"{name}:{age}")
        msg = String()
        msg.data = " ".join(parts)
        self.interface_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlightStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
