import math
import os
import statistics
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, OpticalFlow, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class RectangleFlowTest(Node):
    def __init__(self):
        super().__init__("rectangle_flow_test")
        self.declare_parameter("takeoff_alt", 3.0)
        self.declare_parameter("length_x", 6.0)
        self.declare_parameter("length_y", 4.0)
        self.declare_parameter("speed_mps", 0.8)
        self.declare_parameter("hold_start_s", 5.0)
        self.declare_parameter("hold_corner_s", 2.0)
        self.declare_parameter("setpoint_rate_hz", 15.0)
        self.declare_parameter(
            "setpoint_output_topic", "/autonomy/intent/mission/pose"
        )
        self.declare_parameter("land_at_end", True)
        self.declare_parameter("flow_topic", "/sim/optical_flow/raw")
        self.declare_parameter("min_good_quality", 60)
        self.declare_parameter("preflight_wait_s", 90.0)
        self.declare_parameter("command_retry_s", 45.0)
        self.declare_parameter("mavlink_takeoff_url", "tcp:127.0.0.1:5762")
        self.declare_parameter("takeoff_ack_component", 1)
        self.declare_parameter("takeoff_reached_ratio", 0.55)
        self.declare_parameter("diagnostic_world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("diagnostic_model", "apm_iris")

        self.takeoff_alt = float(self.get_parameter("takeoff_alt").value)
        self.length_x = float(self.get_parameter("length_x").value)
        self.length_y = float(self.get_parameter("length_y").value)
        self.speed_mps = max(0.1, float(self.get_parameter("speed_mps").value))
        self.hold_start_s = float(self.get_parameter("hold_start_s").value)
        self.hold_corner_s = float(self.get_parameter("hold_corner_s").value)
        self.rate_hz = float(self.get_parameter("setpoint_rate_hz").value)
        self.land_at_end = bool(self.get_parameter("land_at_end").value)
        self.min_good_quality = int(self.get_parameter("min_good_quality").value)
        self.preflight_wait_s = float(self.get_parameter("preflight_wait_s").value)
        self.command_retry_s = float(self.get_parameter("command_retry_s").value)
        self.mavlink_takeoff_url = str(self.get_parameter("mavlink_takeoff_url").value)
        self.takeoff_ack_component = int(self.get_parameter("takeoff_ack_component").value)
        self.takeoff_reached_ratio = float(self.get_parameter("takeoff_reached_ratio").value)
        self.diagnostic_world_name = str(self.get_parameter("diagnostic_world_name").value)
        self.diagnostic_model = str(self.get_parameter("diagnostic_model").value)

        self.state = State()
        self.pose = None
        self.global_odom = None
        self.altitude = None
        self.gz_pose = None
        self.flow_samples = []
        self.flow_good = 0
        self.flow_last_time = None
        self.velocity_mav = None

        self.create_subscription(State, "/mavros/state", self._state_cb, 10)
        mavros_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, mavros_sensor_qos
        )
        self.create_subscription(
            Odometry, "/mavros/local_position/odom", self._odom_cb, mavros_sensor_qos
        )
        self.create_subscription(
            Altitude, "/mavros/altitude", self._altitude_cb, mavros_sensor_qos
        )
        self.create_subscription(
            OpticalFlow, self.get_parameter("flow_topic").value, self._flow_cb, 10
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("setpoint_output_topic").value), 10
        )
        self.arming_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_cli = self.create_client(CommandTOL, "/mavros/cmd/land")
        self.gz_node = GzNode()
        self.gz_node.subscribe(
            Pose_V,
            f"/world/{self.diagnostic_world_name}/dynamic_pose/info",
            self._gz_pose_cb,
        )

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg

    def _odom_cb(self, msg):
        self.global_odom = msg

    def _altitude_cb(self, msg):
        self.altitude = msg

    def _gz_pose_cb(self, msg):
        for pose in msg.pose:
            if pose.name == self.diagnostic_model:
                self.gz_pose = pose
                return

    def _flow_cb(self, msg):
        q = int(msg.quality)
        self.flow_samples.append(q)
        if q >= self.min_good_quality:
            self.flow_good += 1
        self.flow_last_time = time.monotonic()
        if len(self.flow_samples) % 50 == 0:
            self.get_logger().info(
                f"flow quality recent={q} median={statistics.median(self.flow_samples[-50:]):.0f} "
                f"good_ratio={self.flow_good / max(1, len(self.flow_samples)):.2f}"
            )

    def wait_ready(self, timeout=60.0):
        self.get_logger().info("Waiting for MAVROS FCU connection...")
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state.connected:
                break
        else:
            raise RuntimeError("Timed out waiting for MAVROS connection")

        for client, name in [
            (self.arming_cli, "arming"),
            (self.mode_cli, "set_mode"),
            (self.takeoff_cli, "takeoff"),
            (self.land_cli, "land"),
        ]:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f"MAVROS service not available: {name}")
        self.get_logger().info(
            f"ready: connected={self.state.connected}, mode={self.state.mode}, armed={self.state.armed}, "
            f"pose={self.pose_summary()}"
        )

    def call(self, client, request, label, timeout=10.0):
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                resp = future.result()
                self.get_logger().info(f"{label}: {resp}")
                return resp
        raise RuntimeError(f"Timeout calling {label}")

    def publish_setpoint(self, x, y, z, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.setpoint_pub.publish(msg)

    def velocity_link(self):
        if self.velocity_mav is None:
            self.velocity_mav = mavutil.mavlink_connection(self.mavlink_takeoff_url, source_system=254)
            self.velocity_mav.wait_heartbeat(timeout=10)
            self.get_logger().info(
                f"MAVLink velocity target: system={self.velocity_mav.target_system or 1}, "
                f"component={self.velocity_mav.target_component or 1}"
            )
        return self.velocity_mav

    def send_velocity_ned(self, vx, vy, vz=0.0, yaw=None, hold_alt=None):
        mav = self.velocity_link()
        target_system = mav.target_system or 1
        target_component = self.takeoff_ack_component or mav.target_component or 1
        time_boot_ms = int((time.monotonic() * 1000.0) % 4294967295)
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        z_down = -float(hold_alt) if hold_alt is not None else 0.0
        if hold_alt is None:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        if yaw is None:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            yaw = 0.0
        mav.mav.set_position_target_local_ned_send(
            time_boot_ms,
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0,
            0.0,
            z_down,
            float(vx),
            float(vy),
            float(vz),
            0.0,
            0.0,
            0.0,
            float(yaw),
            0.0,
        )

    def pose_summary(self):
        parts = []
        if self.pose is not None:
            p = self.pose.pose.position
            parts.append(f"local_pose=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
        else:
            parts.append("local_pose=none")
        if self.global_odom is not None:
            p = self.global_odom.pose.pose.position
            parts.append(f"global_odom=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
        else:
            parts.append("global_odom=none")
        if self.altitude is not None:
            parts.append(
                f"alt(rel={self.altitude.relative:.2f}, local={self.altitude.local:.2f}, amsl={self.altitude.amsl:.2f})"
            )
        else:
            parts.append("alt=none")
        if self.gz_pose is not None:
            parts.append(
                f"gz_pose=({self.gz_pose.position.x:.2f},{self.gz_pose.position.y:.2f},{self.gz_pose.position.z:.2f})"
            )
        else:
            parts.append("gz_pose=none")
        return " ".join(parts)

    def target_position_summary(self):
        if self.pose is not None:
            p = self.pose.pose.position
            return p.x, p.y, p.z
        if self.global_odom is not None:
            p = self.global_odom.pose.pose.position
            return p.x, p.y, p.z
        return float("nan"), float("nan"), float("nan")

    def current_alt(self):
        if self.pose is None:
            if self.global_odom is not None:
                return float(self.global_odom.pose.pose.position.z)
            if self.altitude is not None:
                return float(self.altitude.relative)
            if self.gz_pose is not None:
                return float(self.gz_pose.position.z)
            return float("nan")
        return float(self.pose.pose.position.z)

    def hold(self, x, y, z, seconds, yaw=0.0):
        period = 1.0 / max(self.rate_hz, 1.0)
        end = time.monotonic() + seconds
        next_print = 0.0
        while rclpy.ok() and time.monotonic() < end:
            self.send_velocity_ned(0.0, 0.0, 0.0, yaw=None, hold_alt=z)
            rclpy.spin_once(self, timeout_sec=0.0)
            now = time.monotonic()
            if now >= next_print:
                self.get_logger().info(
                    f"hold velocity=(0.00,0.00,0.00), hold_alt={z:.2f}, nominal=({x:.2f},{y:.2f},{z:.2f}) "
                    f"state mode={self.state.mode} armed={self.state.armed} pose={self.pose_summary()}"
                )
                next_print = now + 2.0
            time.sleep(period)

    def fly_segment(self, start, end, yaw):
        sx, sy, sz = start
        ex, ey, ez = end
        dist = math.sqrt((ex - sx) ** 2 + (ey - sy) ** 2 + (ez - sz) ** 2)
        duration = max(1.0, dist / self.speed_mps)
        steps = max(1, int(duration * self.rate_hz))
        period = 1.0 / max(self.rate_hz, 1.0)
        vx = (ex - sx) / duration
        vy = (ey - sy) / duration
        vz = 0.0
        for i in range(steps + 1):
            t = i / steps
            self.send_velocity_ned(vx, vy, vz, yaw=None, hold_alt=sz)
            rclpy.spin_once(self, timeout_sec=0.0)
            if i == 0 or i == steps or i % max(1, int(self.rate_hz * 2.0)) == 0:
                self.get_logger().info(
                    f"segment progress {i}/{steps}: velocity_ned=({vx:.2f},{vy:.2f},{vz:.2f}), hold_alt={sz:.2f} "
                    f"nominal=({sx + (ex - sx) * t:.2f},{sy + (ey - sy) * t:.2f},{sz + (ez - sz) * t:.2f}) "
                    f"pose={self.pose_summary()}"
                )
            time.sleep(period)
        self.send_velocity_ned(0.0, 0.0, 0.0, yaw=None, hold_alt=ez)

    def wait_for_state(self, predicate, label, timeout=15.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                self.get_logger().info(
                    f"{label} confirmed: mode={self.state.mode}, armed={self.state.armed}, pose={self.pose_summary()}"
                )
                return True
            time.sleep(1.0 / max(self.rate_hz, 1.0))
        self.get_logger().warning(
            f"{label} not confirmed within {timeout:.1f}s: "
            f"mode={self.state.mode}, armed={self.state.armed}, pose={self.pose_summary()}"
        )
        return False

    def set_guided_and_arm(self):
        self.get_logger().info(
            f"Waiting up to {self.preflight_wait_s:.1f}s for EKF/GPS before GUIDED takeoff; "
            "not publishing local setpoints during takeoff preparation"
        )
        mav = None
        try:
            mav = mavutil.mavlink_connection(self.mavlink_takeoff_url, source_system=251)
            mav.wait_heartbeat(timeout=10)
            self.get_logger().info("MAVLink preflight telemetry connection ready")
        except Exception as exc:
            self.get_logger().warning(f"MAVLink preflight telemetry unavailable, falling back to timed wait: {exc}")
            mav = None

        end = time.monotonic() + self.preflight_wait_s
        next_print = 0.0
        gps_ready = False
        global_ready = False
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None or self.global_odom is not None:
                global_ready = True
            if mav is not None:
                for _ in range(20):
                    msg = mav.recv_match(
                        type=["GPS_RAW_INT", "GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"],
                        blocking=False,
                    )
                    if msg is None:
                        break
                    msg_type = msg.get_type()
                    if msg_type == "GPS_RAW_INT" and getattr(msg, "fix_type", 0) >= 3:
                        gps_ready = True
                    elif msg_type == "GLOBAL_POSITION_INT":
                        global_ready = True
                    elif msg_type == "LOCAL_POSITION_NED":
                        global_ready = True
                if global_ready:
                    self.get_logger().info(
                        f"preflight navigation ready: gps_ready={gps_ready}, global/local_position_ready={global_ready}, "
                        f"pose={self.pose_summary()}"
                    )
                    break
            now = time.monotonic()
            if now >= next_print:
                self.get_logger().info(
                    f"preflight state mode={self.state.mode} armed={self.state.armed} "
                    f"gps_ready={gps_ready} global/local_position_ready={global_ready} pose={self.pose_summary()}"
                )
                next_print = now + 2.0
        if mav is not None:
            mav.close()
        if not global_ready:
            self.get_logger().warning(
                "Preflight global/local position was not observed by the state machine; "
                "continuing after the full wait because ArduPilot GPS/EKF readiness can lag MAVROS local topics in SITL. "
                f"Final preflight pose={self.pose_summary()}"
            )
        deadline = time.monotonic() + self.command_retry_s
        while rclpy.ok() and time.monotonic() < deadline:
            if self.state.mode != "GUIDED":
                self.get_logger().info("Sending GUIDED mode request")
                mode_req = SetMode.Request()
                mode_req.base_mode = 0
                mode_req.custom_mode = "GUIDED"
                self.call(self.mode_cli, mode_req, "set GUIDED")
            if not self.wait_for_state(lambda: self.state.mode == "GUIDED", "GUIDED mode", timeout=12.0):
                continue

            if not self.state.armed:
                self.get_logger().info("Sending arm request")
                arm_req = CommandBool.Request()
                arm_req.value = True
                self.call(self.arming_cli, arm_req, "arm")
            if self.wait_for_state(lambda: self.state.armed, "armed state", timeout=12.0):
                return
        raise RuntimeError("Failed to confirm GUIDED + armed state before takeoff")

    def send_takeoff_command_int(self):
        self.get_logger().info(
            f"Sending MAV_CMD_NAV_TAKEOFF COMMAND_INT via {self.mavlink_takeoff_url}, "
            f"target_alt={self.takeoff_alt:.1f}m"
        )
        mav = mavutil.mavlink_connection(self.mavlink_takeoff_url, source_system=252)
        mav.wait_heartbeat(timeout=10)
        target_system = mav.target_system or 1
        target_component = self.takeoff_ack_component or mav.target_component or 1
        self.get_logger().info(f"MAVLink target: system={target_system}, component={target_component}")
        mav.mav.command_int_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0.0,
            0.0,
            1.0,
            0.0,
            0,
            0,
            self.takeoff_alt,
        )
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            msg = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                accepted = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                self.get_logger().info(f"takeoff COMMAND_INT ack: result={msg.result}, accepted={accepted}")
                mav.close()
                return accepted
        mav.close()
        self.get_logger().warning("Timed out waiting for takeoff COMMAND_INT ACK")
        return False

    def wait_takeoff_reached(self, timeout=35.0):
        required = self.takeoff_alt * self.takeoff_reached_ratio
        self.get_logger().info(
            f"Waiting for real climb: require local z >= {required:.2f}m "
            f"({self.takeoff_reached_ratio:.2f} * target)"
        )
        mav = None
        try:
            mav = mavutil.mavlink_connection(self.mavlink_takeoff_url, source_system=253)
            mav.wait_heartbeat(timeout=5)
            self.get_logger().info("MAVLink telemetry connection ready for climb confirmation")
        except Exception as exc:
            self.get_logger().warning(f"MAVLink telemetry unavailable, ROS altitude only: {exc}")
            mav = None

        end = time.monotonic() + timeout
        next_print = 0.0
        mav_alt = float("nan")
        mav_alt_source = "none"
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            alt = self.current_alt()
            if mav is not None:
                for _ in range(10):
                    msg = mav.recv_match(
                        type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED", "VFR_HUD"],
                        blocking=False,
                    )
                    if msg is None:
                        break
                    msg_type = msg.get_type()
                    if msg_type == "GLOBAL_POSITION_INT":
                        mav_alt = float(msg.relative_alt) * 0.001
                        mav_alt_source = "GLOBAL_POSITION_INT.relative_alt"
                    elif msg_type == "LOCAL_POSITION_NED":
                        mav_alt = -float(msg.z)
                        mav_alt_source = "LOCAL_POSITION_NED.-z"
                    elif msg_type == "VFR_HUD":
                        mav_alt = float(msg.alt)
                        mav_alt_source = "VFR_HUD.alt"
                if not math.isfinite(alt) and math.isfinite(mav_alt):
                    alt = mav_alt
            now = time.monotonic()
            if now >= next_print:
                self.get_logger().info(
                    f"climb check: alt={alt:.2f}, mav_alt={mav_alt:.2f}({mav_alt_source}), "
                    f"mode={self.state.mode}, armed={self.state.armed}, pose={self.pose_summary()}"
                )
                next_print = now + 1.0
            if math.isfinite(alt) and alt >= required:
                self.get_logger().info(
                    f"takeoff altitude confirmed: alt={alt:.2f}, mav_alt={mav_alt:.2f}({mav_alt_source}), "
                    f"pose={self.pose_summary()}"
                )
                if mav is not None:
                    mav.close()
                return True
            time.sleep(1.0 / max(self.rate_hz, 1.0))
        if mav is not None:
            mav.close()
        self.get_logger().error(
            f"Takeoff not confirmed. Final state: mode={self.state.mode}, armed={self.state.armed}, "
            f"mav_alt={mav_alt:.2f}({mav_alt_source}), pose={self.pose_summary()}"
        )
        return False

    def takeoff(self):
        if self.send_takeoff_command_int():
            if self.wait_takeoff_reached():
                self.hold(0.0, 0.0, self.takeoff_alt, self.hold_start_s)
                return
            raise RuntimeError("Takeoff command was accepted, but vehicle did not climb")

        self.get_logger().warning("COMMAND_INT takeoff was not accepted; trying MAVROS CommandTOL fallback")
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = self.takeoff_alt
        self.call(self.takeoff_cli, req, f"takeoff fallback {self.takeoff_alt:.1f}m")
        if not self.wait_takeoff_reached():
            raise RuntimeError("MAVROS CommandTOL returned, but vehicle did not climb")

        self.hold(0.0, 0.0, self.takeoff_alt, self.hold_start_s)

    def land(self):
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0
        self.call(self.land_cli, req, "land")

    def report_flow(self):
        n = len(self.flow_samples)
        if n == 0:
            self.get_logger().warning("No optical-flow samples observed on the configured topic.")
            return
        med = statistics.median(self.flow_samples)
        avg = sum(self.flow_samples) / n
        good_ratio = self.flow_good / n
        age = time.monotonic() - self.flow_last_time if self.flow_last_time else float("nan")
        self.get_logger().info(
            f"Optical-flow summary: samples={n}, avg_quality={avg:.1f}, "
            f"median_quality={med:.1f}, good_ratio={good_ratio:.2f}, last_age={age:.2f}s"
        )

    def run(self):
        self.wait_ready()
        self.set_guided_and_arm()
        self.takeoff()

        z = self.takeoff_alt
        points = [
            (0.0, 0.0, z),
            (self.length_x, 0.0, z),
            (self.length_x, self.length_y, z),
            (0.0, self.length_y, z),
            (0.0, 0.0, z),
        ]
        yaws = [0.0, math.pi / 2.0, math.pi, -math.pi / 2.0]
        for i in range(4):
            self.get_logger().info(f"Rectangle edge {i + 1}/4: {points[i]} -> {points[i + 1]}")
            self.fly_segment(points[i], points[i + 1], yaws[i])
            self.hold(*points[i + 1], seconds=self.hold_corner_s, yaw=yaws[i])
            self.report_flow()

        self.report_flow()
        if self.land_at_end:
            self.land()
        else:
            self.get_logger().info("Rectangle complete; holding final setpoint.")
            while rclpy.ok():
                self.hold(0.0, 0.0, z, 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = RectangleFlowTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        if getattr(node, "velocity_mav", None) is not None:
            node.velocity_mav.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
