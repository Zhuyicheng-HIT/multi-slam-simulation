"""Gazebo air-pressure bridge for the simulation-only FCU interface.

The native Gazebo sensor is preferred.  Harmonic versions which do not expose
the native transport publisher use the same Gazebo atmospheric equation with
the model height from Gazebo transport.  The fallback never consumes ROS
ground-truth odometry or publishes a pose-derived navigation factor.
"""

import os

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import math
import time

import numpy as np
import rclpy
from gz.msgs10.fluid_pressure_pb2 import FluidPressure as GzFluidPressure
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure


# Keep the fallback numerically identical to Gazebo Harmonic's
# AirPressureSensor implementation (gz-sensors AirPressureSensor.cc).
_GAS_CONSTANT_NM_PER_KMOL_K = 8314.32
_MEAN_MOLECULAR_AIR_WEIGHT_KG_PER_KMOL = 28.9644
_GRAVITY_M_S2 = 9.80665
_EARTH_RADIUS_M = 6356766.0
_SEA_LEVEL_PRESSURE_PA = 101325.0
_SEA_LEVEL_TEMPERATURE_K = 288.15
_TEMPERATURE_LAPSE_K_PER_M = 0.0065
_AIR_CONSTANT = (
    _GRAVITY_M_S2 * _MEAN_MOLECULAR_AIR_WEIGHT_KG_PER_KMOL
    / (_GAS_CONSTANT_NM_PER_KMOL_K * -_TEMPERATURE_LAPSE_K_PER_M)
)


class GazeboBarometerBridge(Node):
    """Bridge a Gazebo air-pressure sensor into the FCU-compatible ROS topic."""

    def __init__(self):
        super().__init__("gz_barometer_sim")
        self.declare_parameter("world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("model_name", "apm_iris")
        self.declare_parameter("link_name", "front_d435i_link")
        self.declare_parameter("sensor_name", "barometer")
        self.declare_parameter("gz_topic", "")
        self.declare_parameter("sim_topic", "/sim/barometer/pressure")
        self.declare_parameter("ros_topic", "/mavros/imu/static_pressure")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_sim_topic", True)
        self.declare_parameter("publish_ros_topic", False)
        self.declare_parameter("fallback_from_pose", True)
        self.declare_parameter("fallback_timeout_s", 1.0)
        self.declare_parameter("ground_z_m", 0.0)
        self.declare_parameter("reference_pressure_pa", _SEA_LEVEL_PRESSURE_PA)
        self.declare_parameter("reference_altitude_m", 584.0)
        self.declare_parameter("fallback_noise_std_pa", 2.0)
        self.declare_parameter("noise_seed", 2718)

        world = str(self.get_parameter("world_name").value)
        model = str(self.get_parameter("model_name").value)
        link = str(self.get_parameter("link_name").value)
        sensor = str(self.get_parameter("sensor_name").value)
        configured_topic = str(self.get_parameter("gz_topic").value).strip()
        self.gz_topic = configured_topic or (
            f"/world/{world}/model/{model}/link/{link}/sensor/"
            f"{sensor}/air_pressure"
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.sim_topic = str(self.get_parameter("sim_topic").value)
        self.ros_topic = str(self.get_parameter("ros_topic").value)
        self.publish_sim_topic = bool(self.get_parameter("publish_sim_topic").value)
        self.publish_ros_topic = bool(self.get_parameter("publish_ros_topic").value)
        self.fallback_from_pose = bool(self.get_parameter("fallback_from_pose").value)
        self.fallback_timeout_s = float(self.get_parameter("fallback_timeout_s").value)
        self.ground_z_m = float(self.get_parameter("ground_z_m").value)
        self.reference_pressure_pa = float(self.get_parameter("reference_pressure_pa").value)
        configured_reference_altitude = float(
            self.get_parameter("reference_altitude_m").value
        )
        self.reference_altitude_m = configured_reference_altitude
        self.fallback_noise_std_pa = float(self.get_parameter("fallback_noise_std_pa").value)
        self.rng = np.random.default_rng(
            int(self.get_parameter("noise_seed").value)
        )
        if self.fallback_timeout_s <= 0.0:
            raise ValueError("fallback_timeout_s must be positive")
        if not math.isfinite(self.reference_altitude_m):
            raise ValueError("reference_altitude_m must be finite")
        if not self.publish_sim_topic and not self.publish_ros_topic:
            raise ValueError("at least one pressure output topic must be enabled")

        self.sim_pub = (
            self.create_publisher(FluidPressure, self.sim_topic, qos_profile_sensor_data)
            if self.publish_sim_topic
            else None
        )
        self.ros_pub = (
            self.create_publisher(FluidPressure, self.ros_topic, qos_profile_sensor_data)
            if self.publish_ros_topic
            else None
        )
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzFluidPressure, self.gz_topic, self._pressure_cb)
        self.last_native_wall_time = None
        self.latest_pose = None
        self.last_fallback_stamp = None
        self.fallback_count = 0
        if self.fallback_from_pose:
            pose_topics = (
                f"/world/{world}/dynamic_pose/info",
                f"/world/{world}/pose/info",
            )
            for pose_topic in pose_topics:
                self.gz_node.subscribe(Pose_V, pose_topic, self._pose_cb)
        self.received = 0
        self.invalid = 0
        self.last_report_time = self.get_clock().now().nanoseconds * 1.0e-9
        self.get_logger().info(
            f"Gazebo barometer active: {self.gz_topic} -> "
            f"{self.sim_topic if self.sim_pub else ''}"
            f"{', ' if self.sim_pub and self.ros_pub else ''}"
            f"{self.ros_topic if self.ros_pub else ''}"
        )
        self.get_logger().info(
            "Barometer pressure datum fixed from world spherical elevation: "
            f"reference_altitude_m={self.reference_altitude_m:.3f}, "
            f"reference_pressure_pa={self.reference_pressure_pa:.3f}"
        )
        self.create_timer(0.05, self._fallback_timer)
        self.create_timer(5.0, self._report)

    @staticmethod
    def _stamp(message):
        try:
            stamp = message.header.stamp
            return int(stamp.sec), int(stamp.nsec)
        except (AttributeError, TypeError, ValueError):
            return None

    def _pressure_cb(self, message):
        pressure = float(message.pressure)
        variance = float(message.variance)
        if not math.isfinite(pressure) or not 30000.0 <= pressure <= 120000.0:
            self.invalid += 1
            return
        if not math.isfinite(variance) or variance < 0.0:
            variance = 0.0

        output = FluidPressure()
        stamp = self._stamp(message)
        if stamp is None:
            output.header.stamp = self.get_clock().now().to_msg()
        else:
            output.header.stamp.sec = stamp[0]
            output.header.stamp.nanosec = stamp[1]
        output.header.frame_id = self.frame_id
        output.fluid_pressure = pressure
        output.variance = variance
        if self.sim_pub is not None:
            self.sim_pub.publish(output)
        if self.ros_pub is not None:
            self.ros_pub.publish(output)
        self.received += 1
        self.last_native_wall_time = time.monotonic()

    def _pose_cb(self, message):
        try:
            stamp = message.header.stamp
            stamp_s = float(stamp.sec) + float(stamp.nsec) * 1.0e-9
        except (AttributeError, TypeError, ValueError):
            return
        model_name = str(self.get_parameter("model_name").value)
        for pose in message.pose:
            if pose.name == model_name or pose.name.endswith(f"::{model_name}"):
                self.latest_pose = (stamp_s, float(pose.position.z))
                return

    def _publish(self, stamp_s, pressure, variance):
        output = FluidPressure()
        sec = int(stamp_s)
        nanosec = int(round((stamp_s - sec) * 1.0e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        output.header.stamp.sec = sec
        output.header.stamp.nanosec = nanosec
        output.header.frame_id = self.frame_id
        output.fluid_pressure = float(pressure)
        output.variance = float(max(0.0, variance))
        if self.sim_pub is not None:
            self.sim_pub.publish(output)
        if self.ros_pub is not None:
            self.ros_pub.publish(output)
        self.fallback_count += 1

    @staticmethod
    def pressure_from_height(
        reference_altitude_m, local_height_m, reference_pressure_pa=_SEA_LEVEL_PRESSURE_PA
    ):
        """Return pressure using Gazebo Harmonic's air-pressure model."""
        height_m = float(reference_altitude_m) + float(local_height_m)
        geo_height_m = _EARTH_RADIUS_M * height_m / (_EARTH_RADIUS_M + height_m)
        temperature_k = _SEA_LEVEL_TEMPERATURE_K - (
            _TEMPERATURE_LAPSE_K_PER_M * geo_height_m
        )
        if temperature_k <= 0.0:
            raise ValueError("air-pressure model temperature became non-positive")
        return float(
            reference_pressure_pa
            * math.exp(
                _AIR_CONSTANT
                * math.log(_SEA_LEVEL_TEMPERATURE_K / temperature_k)
            )
        )

    def _fallback_timer(self):
        if (
            not self.fallback_from_pose
            or self.latest_pose is None
        ):
            return
        if (
            self.last_native_wall_time is not None
            and time.monotonic() - self.last_native_wall_time <= self.fallback_timeout_s
        ):
            return
        stamp_s, z_m = self.latest_pose
        if self.last_fallback_stamp is not None and stamp_s <= self.last_fallback_stamp:
            return
        height_m = max(0.0, z_m - self.ground_z_m)
        pressure = self.pressure_from_height(
            self.reference_altitude_m,
            height_m,
            self.reference_pressure_pa,
        )
        pressure += float(self.rng.normal(0.0, self.fallback_noise_std_pa))
        self._publish(stamp_s, pressure, self.fallback_noise_std_pa ** 2)
        self.last_fallback_stamp = stamp_s

    def _report(self):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        self.get_logger().info(
            f"barometer samples: native={self.received}, fallback={self.fallback_count}, "
            f"invalid={self.invalid}, reference_altitude_m="
            f"{self.reference_altitude_m:.3f}, "
            f"wall_since_report={now - self.last_report_time:.2f}s"
        )
        self.last_report_time = now

    def destroy_node(self):
        try:
            self.gz_node.unsubscribe(self.gz_topic)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBarometerBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
