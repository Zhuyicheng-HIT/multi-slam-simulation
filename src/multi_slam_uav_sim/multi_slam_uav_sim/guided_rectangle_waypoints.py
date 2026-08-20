import math
import time

from pymavlink import mavutil

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros.mavlink import convert_to_bytes
from mavros_msgs.msg import Mavlink, OpticalFlow, State, StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


def ekf_flags_have_absolute_position(flags):
    flags = int(flags)
    required = (
        mavutil.mavlink.EKF_ATTITUDE
        | mavutil.mavlink.EKF_VELOCITY_HORIZ
        | mavutil.mavlink.EKF_POS_HORIZ_ABS
    )
    invalid = (
        mavutil.mavlink.EKF_GPS_GLITCHING
        | mavutil.mavlink.EKF_UNINITIALIZED
    )
    return flags & required == required and flags & invalid == 0


class GuidedRectangleWaypoints(Node):
    """GPS/GUIDED rectangle flight using MAVROS local-position setpoints."""

    def __init__(self, node_name="guided_rectangle_waypoints"):
        super().__init__(node_name)
        self.declare_parameter("takeoff_alt", 3.0)
        self.declare_parameter("length_x", 2.0)
        self.declare_parameter("length_y", 1.2)
        self.declare_parameter("speed_mps", 0.20)
        self.declare_parameter("hold_time", 2.0)
        self.declare_parameter("post_takeoff_hold_time_s", 3.0)
        self.declare_parameter("yaw_rate_deg_s", 12.0)
        self.declare_parameter("face_rectangle_edges", True)
        self.declare_parameter("setpoint_rate_hz", 10.0)
        self.declare_parameter("preflight_wait_s", 45.0)
        self.declare_parameter("navigation_stable_s", 1.0)
        self.declare_parameter("navigation_source", "auto")
        self.declare_parameter("flow_topic", "/mavros/optical_flow/raw/optical_flow")
        self.declare_parameter("mavlink_source_topic", "/uas1/mavlink_source")
        self.declare_parameter("flow_min_quality", 0)
        self.declare_parameter("flow_max_age_s", 1.0)
        self.declare_parameter("command_retry_s", 60.0)
        self.declare_parameter("mavros_disconnect_grace_s", 2.0)
        self.declare_parameter("land_at_end", True)
        self.declare_parameter("land_disarm_timeout_s", 60.0)
        self.declare_parameter("final_hold_time_s", 0.0)
        # SERIAL1/5762 is reserved for the MTF01P. Use the independent
        # SERIAL2 MAVLink endpoint for direct COMMAND_INT acknowledgements.
        self.declare_parameter("mavlink_takeoff_url", "tcp:127.0.0.1:5763")
        self.declare_parameter("mavlink_target_component", 1)
        self.declare_parameter("takeoff_param3", 1.0)
        self.declare_parameter("takeoff_free_climb_s", 14.0)
        self.declare_parameter("takeoff_command_attempts", 2)
        self.declare_parameter("takeoff_min_alt_fraction", 0.45)
        self.declare_parameter("takeoff_min_alt_m", 0.7)

        self.takeoff_alt = float(self.get_parameter("takeoff_alt").value)
        self.length_x = float(self.get_parameter("length_x").value)
        self.length_y = float(self.get_parameter("length_y").value)
        self.speed_mps = max(0.05, float(self.get_parameter("speed_mps").value))
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.post_takeoff_hold_time_s = max(
            0.0, float(self.get_parameter("post_takeoff_hold_time_s").value)
        )
        self.yaw_rate = math.radians(max(1.0, float(self.get_parameter("yaw_rate_deg_s").value)))
        self.face_rectangle_edges = bool(
            self.get_parameter("face_rectangle_edges").value
        )
        self.land_disarm_timeout_s = float(
            self.get_parameter("land_disarm_timeout_s").value
        )
        self.rate_hz = max(1.0, float(self.get_parameter("setpoint_rate_hz").value))
        self.preflight_wait_s = float(self.get_parameter("preflight_wait_s").value)
        self.navigation_stable_s = float(self.get_parameter("navigation_stable_s").value)
        self.navigation_source = str(self.get_parameter("navigation_source").value).lower()
        if self.navigation_source not in ("auto", "gps", "optical_flow"):
            raise ValueError("navigation_source must be auto, gps, or optical_flow")
        self.flow_topic = str(self.get_parameter("flow_topic").value)
        self.flow_min_quality = int(self.get_parameter("flow_min_quality").value)
        self.flow_max_age_s = float(self.get_parameter("flow_max_age_s").value)
        self.command_retry_s = float(self.get_parameter("command_retry_s").value)
        self.mavros_disconnect_grace_s = max(
            0.0,
            float(self.get_parameter("mavros_disconnect_grace_s").value),
        )
        self.land_at_end = bool(self.get_parameter("land_at_end").value)
        self.final_hold_time_s = max(
            0.0, float(self.get_parameter("final_hold_time_s").value)
        )
        self.mavlink_takeoff_url = str(self.get_parameter("mavlink_takeoff_url").value)
        self.mavlink_target_component = int(self.get_parameter("mavlink_target_component").value)
        self.takeoff_param3 = float(self.get_parameter("takeoff_param3").value)
        self.takeoff_free_climb_s = float(self.get_parameter("takeoff_free_climb_s").value)
        self.takeoff_command_attempts = max(
            1, int(self.get_parameter("takeoff_command_attempts").value)
        )
        self.takeoff_min_alt_fraction = float(self.get_parameter("takeoff_min_alt_fraction").value)
        self.takeoff_min_alt_m = float(self.get_parameter("takeoff_min_alt_m").value)

        self.state = State()
        self.pose = None
        self.fix = None
        self.last_statustext = ""
        self.ekf_using_gps = False
        self.ekf_absolute_position_ready = False
        self.mavlink_parser = mavutil.mavlink.MAVLink(None)
        self.last_flow_quality = 0
        self.last_flow_time = None
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.home_yaw = 0.0
        self.last_status_time = 0.0
        self.last_commanded_setpoint = None
        self.last_connected_guided_armed_wall_s = None
        self.disconnect_grace_logged = False

        self.create_subscription(State, "/mavros/state", self._state_cb, 10)
        self.create_subscription(
            StatusText, "/mavros/statustext/recv", self._status_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Mavlink,
            str(self.get_parameter("mavlink_source_topic").value),
            self._mavlink_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._fix_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OpticalFlow,
            self.flow_topic,
            self._flow_cb,
            qos_profile_sensor_data,
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self.arming_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_cli = self.create_client(CommandTOL, "/mavros/cmd/land")
        self.mission_phase_pub = self.create_publisher(String, "/mission/phase", 10)

    def _publish_mission_phase(self, phase):
        message = String()
        message.data = str(phase)
        self.mission_phase_pub.publish(message)
        self.get_logger().info(f"Mission phase: {message.data}")

    def _state_cb(self, msg):
        self.state = msg
        if msg.connected and msg.armed and msg.mode == "GUIDED":
            self.last_connected_guided_armed_wall_s = time.monotonic()
            self.disconnect_grace_logged = False

    def _pose_cb(self, msg):
        self.pose = msg

    def _fix_cb(self, msg):
        self.fix = msg

    def _status_cb(self, msg):
        self.last_statustext = msg.text
        if "using GPS" in msg.text:
            self.ekf_using_gps = True
        self.get_logger().info(
            f"FCU: {msg.text}; wall_epoch_s={time.time():.6f}; "
            f"wall_monotonic_s={time.monotonic():.6f}"
        )

    def _mavlink_cb(self, msg):
        try:
            decoded = self.mavlink_parser.parse_char(convert_to_bytes(msg))
        except Exception:
            return
        if decoded is None or decoded.get_type() != "EKF_STATUS_REPORT":
            return
        self.ekf_absolute_position_ready = ekf_flags_have_absolute_position(
            decoded.flags
        )

    def _flow_cb(self, msg):
        self.last_flow_quality = int(msg.quality)
        stamp = msg.header.stamp
        self.last_flow_time = (
            float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _wait_until_sim_time(self, target_s, previous_s):
        """Yield callbacks until ROS simulation time reaches the next setpoint tick."""
        observed_s = float(previous_s)
        while rclpy.ok():
            now_s = self._now_s()
            if now_s + 1.0e-9 < observed_s:
                raise RuntimeError("ROS clock moved backwards during mission")
            if now_s >= target_s:
                return now_s
            observed_s = now_s
            rclpy.spin_once(self, timeout_sec=0.01)
        return observed_s

    def _gps_ready(self):
        return self.fix is not None and self.fix.status.status >= 0

    def _gps_navigation_ready(self):
        # A valid receiver fix does not prove that EKF3 has accepted GPS as a
        # navigation source. Arming before that transition can leave a GUIDED
        # takeoff acknowledged while the motors remain at idle.
        return self._gps_ready() and (
            self.ekf_using_gps or self.ekf_absolute_position_ready
        )

    def _flow_ready(self):
        if self.last_flow_time is None:
            return False
        age_s = self._now_s() - self.last_flow_time
        return (
            0.0 <= age_s <= self.flow_max_age_s
            and self.last_flow_quality >= self.flow_min_quality
        )

    def _navigation_source(self):
        if self.navigation_source in ("auto", "gps") and self._gps_navigation_ready():
            return "gps"
        if self.navigation_source in ("auto", "optical_flow") and self._flow_ready():
            return "optical_flow"
        return None

    def _pose_text(self):
        if self.pose is None:
            return "local_pose=none"
        p = self.pose.pose.position
        return f"local_pose=({p.x:.2f},{p.y:.2f},{p.z:.2f})"

    def _local_z(self):
        if self.pose is None:
            return None
        return float(self.pose.pose.position.z)

    def _log_status(self, prefix):
        now = self._now_s()
        if now < self.last_status_time:
            self.last_status_time = now - 2.0
        if now - self.last_status_time < 2.0:
            return
        self.last_status_time = now
        self.get_logger().info(
            f"{prefix}: connected={self.state.connected} mode={self.state.mode} "
            f"armed={self.state.armed} gps_fix_msg={self._gps_ready()} "
            f"ekf_using_gps={self.ekf_using_gps} "
            f"ekf_abs_position={self.ekf_absolute_position_ready} "
            f"flow_quality={self.last_flow_quality} "
            f"navigation_source={self._navigation_source() or 'none'} {self._pose_text()}"
        )

    def wait_ready(self):
        self.get_logger().info("Waiting for MAVROS connection before GPS/GUIDED flight...")
        end = time.monotonic() + self.preflight_wait_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            self._log_status("preflight")
            if self.state.connected:
                break
        else:
            raise RuntimeError("Preflight failed: MAVROS was not connected")

        for client, name in [
            (self.arming_cli, "arming"),
            (self.mode_cli, "set_mode"),
            (self.takeoff_cli, "takeoff"),
        ]:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f"MAVROS service not available: {name}")
        self.get_logger().info(
            "MAVROS services ready. Waiting for local pose and either GPS or optical flow."
        )

    def wait_navigation_ready(self):
        # Bound startup with wall time because /clock can still be zero here and
        # then jump to Gazebo's current time on the first callback.  Navigation
        # stability itself remains measured in ROS time.
        deadline = time.monotonic() + self.preflight_wait_s
        stable_source = None
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            source = self._navigation_source() if self.pose is not None else None
            self._log_status("navigation readiness")
            if source is None:
                stable_source = None
                stable_since = None
                continue
            now = self._now_s()
            if source != stable_source:
                stable_source = source
                stable_since = now
            if now - stable_since < self.navigation_stable_s:
                continue
            p = self.pose.pose.position
            self.home_x = float(p.x)
            self.home_y = float(p.y)
            self.home_z = float(p.z)
            q = self.pose.pose.orientation
            self.home_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self.get_logger().info(
                f"Navigation ready from {source} after a stable {self.navigation_stable_s:.1f}s; "
                f"local origin=({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}), "
                f"yaw={math.degrees(self.home_yaw):.1f}deg."
            )
            return source
        raise RuntimeError(
            "Navigation readiness timed out: require MAVROS local pose and either "
            f"requested source={self.navigation_source}, GPS accepted by EKF3 "
            f"(fix_valid={self._gps_ready()}, ekf_using_gps={self.ekf_using_gps}, "
            f"ekf_abs_position={self.ekf_absolute_position_ready}) "
            f"or fresh optical flow quality >= {self.flow_min_quality}"
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
        client.remove_pending_request(future)
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
        self.last_commanded_setpoint = (
            float(x), float(y), float(z), float(yaw)
        )

    def mission_safety_checkpoint(self, label):
        """Hook for missions that supervise estimator health while moving."""

    def activate_route_control(self):
        """Hook for diagnostic controllers that adapt another feedback frame."""

    def ensure_guided(self, label):
        if not self.state.connected:
            elapsed_s = math.inf
            if self.last_connected_guided_armed_wall_s is not None:
                elapsed_s = (
                    time.monotonic()
                    - self.last_connected_guided_armed_wall_s
                )
            if elapsed_s <= self.mavros_disconnect_grace_s:
                if not self.disconnect_grace_logged:
                    self.get_logger().warning(
                        f"{label}: MAVROS heartbeat temporarily unavailable; "
                        f"holding the active setpoint for up to "
                        f"{self.mavros_disconnect_grace_s:.1f}s"
                    )
                    self.disconnect_grace_logged = True
                return
            raise RuntimeError(
                f"{label}: MAVROS disconnected during an active flight "
                f"segment for {elapsed_s:.2f}s; "
                f"last_fcu_text={self.last_statustext}"
            )
        if not self.state.armed:
            raise RuntimeError(
                f"{label}: FCU disarmed during an active flight segment; "
                f"mode={self.state.mode}, last_fcu_text={self.last_statustext}"
            )
        if self.state.mode != "GUIDED":
            raise RuntimeError(
                f"{label}: FCU left GUIDED mode; current mode={self.state.mode}, "
                f"armed={self.state.armed}, last_fcu_text={self.last_statustext}"
            )

    def hold_setpoint(self, x, y, z, seconds, yaw=None, label="hold", require_guided=False):
        yaw = self.home_yaw if yaw is None else yaw
        period = 1.0 / self.rate_hz
        start_ros_s = self._now_s()
        end_ros_s = start_ros_s + max(0.0, seconds)
        next_publish_ros_s = start_ros_s
        last_observed_ros_s = start_ros_s
        while rclpy.ok() and self._now_s() < end_ros_s:
            if self._now_s() < start_ros_s:
                raise RuntimeError("ROS clock moved backwards during hold")
            if require_guided:
                self.ensure_guided(label)
            self.mission_safety_checkpoint(label)
            self.publish_setpoint(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            next_publish_ros_s = max(
                next_publish_ros_s + period, self._now_s()
            )
            last_observed_ros_s = self._wait_until_sim_time(
                min(next_publish_ros_s, end_ros_s), last_observed_ros_s
            )

    def spin_without_setpoints(self, seconds, label="free climb"):
        start_ros_s = self._now_s()
        end_ros_s = start_ros_s + max(0.0, seconds)
        while rclpy.ok() and self._now_s() < end_ros_s:
            if self._now_s() < start_ros_s:
                raise RuntimeError("ROS clock moved backwards during free climb")
            rclpy.spin_once(self, timeout_sec=0.1)
            self._log_status(label)

    def wait_for_takeoff_climb(self):
        min_takeoff_z = max(
            self.takeoff_min_alt_m,
            self.takeoff_alt * self.takeoff_min_alt_fraction,
        )
        for attempt in range(1, self.takeoff_command_attempts + 1):
            start_ros_s = self._now_s()
            deadline = start_ros_s + self.takeoff_free_climb_s
            stable_since = None
            while rclpy.ok() and self._now_s() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                self._log_status(f"apm free climb attempt {attempt}")
                local_z = self._local_z()
                if local_z is not None and local_z >= min_takeoff_z:
                    stable_since = stable_since or self._now_s()
                    if self._now_s() - stable_since >= 0.5:
                        self.get_logger().info(
                            f"Takeoff climb confirmed at local_z={local_z:.2f}m "
                            f"(required>={min_takeoff_z:.2f}m, attempt={attempt})."
                        )
                        return
                else:
                    stable_since = None
            if attempt >= self.takeoff_command_attempts:
                break
            if not self.state.armed or self.state.mode != "GUIDED":
                break
            self.get_logger().warning(
                f"Takeoff ACK produced no climb in {self.takeoff_free_climb_s:.1f}s; "
                f"re-sending once (attempt {attempt + 1}/{self.takeoff_command_attempts})."
            )
            if not self.send_takeoff_command_int():
                break
        if self.state.armed:
            try:
                request = CommandBool.Request()
                request.value = False
                self.call(self.arming_cli, request, "safety disarm after failed takeoff")
            except Exception as exc:
                self.get_logger().error(f"Safety disarm failed: {exc}")
        raise RuntimeError(
            "Takeoff did not produce FCU local-position climb before timeout: "
            f"local_z={self._local_z() if self._local_z() is not None else 'none'}, "
            f"required>={min_takeoff_z:.2f}m, target={self.takeoff_alt:.2f}m"
        )

    def wait_for_state(self, predicate, label, timeout=12.0):
        end = time.monotonic() + timeout
        next_publish_ros_s = self._now_s()
        while rclpy.ok() and time.monotonic() < end:
            now_ros_s = self._now_s()
            if now_ros_s >= next_publish_ros_s:
                self.publish_setpoint(
                    self.home_x, self.home_y, self.takeoff_alt, self.home_yaw
                )
                next_publish_ros_s = now_ros_s + 1.0 / self.rate_hz
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                self.get_logger().info(
                    f"{label} confirmed: mode={self.state.mode}, "
                    f"armed={self.state.armed}, connected={self.state.connected}"
                )
                return True
        self.get_logger().warning(
            f"{label} not confirmed within {timeout:.1f}s: mode={self.state.mode}, "
            f"armed={self.state.armed}, connected={self.state.connected}, "
            f"last_fcu_text={self.last_statustext}"
        )
        return False

    def send_takeoff_command_int(self):
        self.get_logger().info(
            f"Sending MAV_CMD_NAV_TAKEOFF COMMAND_INT via {self.mavlink_takeoff_url}, "
            f"alt={self.takeoff_alt:.1f}m param3={self.takeoff_param3:.1f}"
        )
        mav = mavutil.mavlink_connection(self.mavlink_takeoff_url, source_system=252)
        mav.wait_heartbeat(timeout=10)
        target_system = mav.target_system or 1
        target_component = self.mavlink_target_component or mav.target_component or 1
        mav.mav.command_int_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0.0,
            0.0,
            self.takeoff_param3,
            0.0,
            0,
            0,
            self.takeoff_alt,
        )
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
            if ack is None:
                continue
            if ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                accepted = ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                self.get_logger().info(f"takeoff COMMAND_INT ack: result={ack.result}, accepted={accepted}")
                mav.close()
                return accepted
        mav.close()
        self.get_logger().warning("Timed out waiting for takeoff COMMAND_INT ACK")
        return False

    def send_takeoff_command_tol(self):
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = self.takeoff_alt
        resp = self.call(self.takeoff_cli, req, f"takeoff CommandTOL {self.takeoff_alt:.1f}m")
        return bool(getattr(resp, "success", False))

    def set_guided_arm_takeoff(self):
        deadline = time.monotonic() + self.command_retry_s
        guided_ok = False
        armed_ok = False
        takeoff_ok = False
        while rclpy.ok() and time.monotonic() < deadline:
            if self.state.mode != "GUIDED":
                mode_req = SetMode.Request()
                mode_req.base_mode = 0
                mode_req.custom_mode = "GUIDED"
                try:
                    resp = self.call(self.mode_cli, mode_req, "set GUIDED")
                except RuntimeError as exc:
                    self.get_logger().warning(
                        f"{exc}; checking FCU state before retrying GUIDED."
                    )
                    guided_ok = self.wait_for_state(
                        lambda: self.state.mode == "GUIDED",
                        "GUIDED after service timeout",
                        timeout=3.0,
                    )
                    if not guided_ok:
                        continue
                else:
                    if not bool(getattr(resp, "mode_sent", False)):
                        self.get_logger().warning(
                            "GUIDED mode command was not accepted for sending; retrying."
                        )
                        self.hold_setpoint(
                            self.home_x, self.home_y, self.takeoff_alt, 2.0,
                            label="guided retry",
                        )
                        continue
            guided_ok = self.wait_for_state(
                lambda: self.state.mode == "GUIDED", "GUIDED mode", timeout=15.0
            )
            if not guided_ok:
                self.hold_setpoint(
                    self.home_x, self.home_y, self.takeoff_alt, 2.0,
                    label="guided wait",
                )
                continue

            if not self.state.armed:
                arm_req = CommandBool.Request()
                arm_req.value = True
                try:
                    resp = self.call(self.arming_cli, arm_req, "arm")
                except RuntimeError as exc:
                    self.get_logger().warning(
                        f"{exc}; checking FCU state before retrying arm."
                    )
                    armed_ok = self.wait_for_state(
                        lambda: self.state.armed,
                        "armed after service timeout",
                        timeout=3.0,
                    )
                    if not armed_ok:
                        continue
                else:
                    if not bool(getattr(resp, "success", False)):
                        self.get_logger().warning("Arm rejected; waiting and retrying.")
                        self.hold_setpoint(
                            self.home_x, self.home_y, self.takeoff_alt, 3.0,
                            label="arm retry",
                        )
                        continue
            armed_ok = self.wait_for_state(
                lambda: self.state.armed, "armed state", timeout=15.0
            )
            if not armed_ok:
                self.hold_setpoint(self.home_x, self.home_y, self.takeoff_alt, 2.0, label="arm wait")
                continue

            if self.state.mode != "GUIDED":
                self.get_logger().warning(f"Mode changed to {self.state.mode}; retrying GUIDED before takeoff.")
                continue

            takeoff_ok = self.send_takeoff_command_int()
            if takeoff_ok:
                self.get_logger().info("Takeoff COMMAND_INT accepted after GUIDED + armed state.")
                return
            self.get_logger().warning("Takeoff rejected; waiting and retrying.")
            self.hold_setpoint(self.home_x, self.home_y, self.takeoff_alt, 3.0, label="takeoff retry")
        raise RuntimeError(
            f"Command phase failed: guided={guided_ok}, armed={armed_ok}, takeoff={takeoff_ok}"
        )

    def fly_segment(self, start, goal, yaw, label):
        sx, sy, sz = start
        gx, gy, gz = goal
        dist = math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2 + (gz - sz) ** 2)
        duration = max(dist / self.speed_mps, 1.0)
        self.get_logger().info(
            f"{label}: ({sx:.2f},{sy:.2f},{sz:.2f}) -> "
            f"({gx:.2f},{gy:.2f},{gz:.2f}), duration={duration:.1f}s"
        )
        started_ros_s = self._now_s()
        next_publish_ros_s = started_ros_s
        last_observed_ros_s = started_ros_s
        while rclpy.ok():
            if not rclpy.ok():
                return
            now_ros_s = self._now_s()
            if now_ros_s < started_ros_s:
                raise RuntimeError("ROS clock moved backwards during flight segment")
            self.ensure_guided(label)
            self.mission_safety_checkpoint(label)
            a = min(1.0, (now_ros_s - started_ros_s) / duration)
            x = sx + (gx - sx) * a
            y = sy + (gy - sy) * a
            z = sz + (gz - sz) * a
            self.publish_setpoint(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            if a >= 1.0:
                break
            next_publish_ros_s = max(
                next_publish_ros_s + 1.0 / self.rate_hz, self._now_s()
            )
            last_observed_ros_s = self._wait_until_sim_time(
                next_publish_ros_s, last_observed_ros_s
            )
        self.hold_setpoint(gx, gy, gz, self.hold_time, yaw, label=f"{label} hold", require_guided=True)

    def rotate_in_place(self, position, start_yaw, goal_yaw, label):
        delta = math.atan2(math.sin(goal_yaw - start_yaw), math.cos(goal_yaw - start_yaw))
        duration = abs(delta) / self.yaw_rate
        self.get_logger().info(
            f"{label}: yaw {math.degrees(start_yaw):.1f} -> "
            f"{math.degrees(start_yaw + delta):.1f} deg, duration={duration:.1f}s"
        )
        started_ros_s = self._now_s()
        next_publish_ros_s = started_ros_s
        last_observed_ros_s = started_ros_s
        while rclpy.ok():
            now_ros_s = self._now_s()
            if now_ros_s < started_ros_s:
                raise RuntimeError("ROS clock moved backwards during yaw segment")
            self.ensure_guided(label)
            self.mission_safety_checkpoint(label)
            progress = 1.0 if duration <= 0.0 else min(
                1.0, (now_ros_s - started_ros_s) / duration
            )
            yaw = start_yaw + delta * progress
            self.publish_setpoint(*position, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            if progress >= 1.0:
                break
            next_publish_ros_s = max(
                next_publish_ros_s + 1.0 / self.rate_hz, self._now_s()
            )
            last_observed_ros_s = self._wait_until_sim_time(
                next_publish_ros_s, last_observed_ros_s
            )
        self.hold_setpoint(*position, 1.0, goal_yaw, label=f"{label} settle", require_guided=True)

    def run(self):
        self._publish_mission_phase("preflight")
        self.wait_ready()
        navigation_source = self.wait_navigation_ready()
        z = self.takeoff_alt
        start = (self.home_x, self.home_y, z)

        self.get_logger().info(
            f"Preflight accepted using {navigation_source}; entering GUIDED/arm/takeoff."
        )
        self.set_guided_arm_takeoff()

        self.get_logger().info(
            f"Takeoff accepted; letting ArduPilot climb without local setpoints for {self.takeoff_free_climb_s:.1f}s."
        )
        self.wait_for_takeoff_climb()

        self.get_logger().info("Switching to local setpoints for rectangle hold/waypoints...")
        self.ensure_guided("post-takeoff")
        self._publish_mission_phase("post_takeoff_hold")
        self.hold_setpoint(
            *start, seconds=self.post_takeoff_hold_time_s, yaw=self.home_yaw,
            label="post-takeoff hold", require_guided=True)
        self.activate_route_control()

        edge_yaws = (
            [self.home_yaw, self.home_yaw + math.pi / 2.0,
             self.home_yaw + math.pi, self.home_yaw + 3.0 * math.pi / 2.0]
            if self.face_rectangle_edges else [self.home_yaw] * 4
        )
        points = [
            (self.home_x, self.home_y, z, self.home_yaw),
            (self.home_x + self.length_x, self.home_y, z, edge_yaws[0]),
            (self.home_x + self.length_x, self.home_y + self.length_y, z, edge_yaws[1]),
            (self.home_x, self.home_y + self.length_y, z, edge_yaws[2]),
            (self.home_x, self.home_y, z, edge_yaws[3]),
        ]
        current = points[0][:3]
        current_yaw = points[0][3]
        self._publish_mission_phase("route_active")
        for idx, (x, y, target_z, yaw) in enumerate(points[1:], start=1):
            goal = (x, y, target_z)
            if abs(math.atan2(math.sin(yaw - current_yaw), math.cos(yaw - current_yaw))) > math.radians(1.0):
                self.rotate_in_place(current, current_yaw, yaw, f"turn {idx}/4")
            self.fly_segment(current, goal, yaw, f"waypoint {idx}/4")
            current = goal
            current_yaw = yaw

        if self.final_hold_time_s > 0.0:
            self._publish_mission_phase("final_loop_hold")
            self.hold_setpoint(
                *current, seconds=self.final_hold_time_s, yaw=points[-1][3],
                label="final loop-closure hold", require_guided=True)

        if self.land_at_end:
            self._publish_mission_phase("landing")
            if self.land_cli.wait_for_service(timeout_sec=5.0):
                land_req = CommandTOL.Request()
                land_req.min_pitch = 0.0
                land_req.yaw = 0.0
                land_req.latitude = 0.0
                land_req.longitude = 0.0
                land_req.altitude = 0.0
                response = self.call(self.land_cli, land_req, "land")
                if not bool(getattr(response, "success", False)):
                    raise RuntimeError("LAND command was rejected by the FCU")
                deadline = time.monotonic() + self.land_disarm_timeout_s
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    self._log_status("landing descent")
                    if not self.state.armed:
                        self.get_logger().info(
                            "LAND completed and FCU disarm confirmed."
                        )
                        self._publish_mission_phase("landed")
                        break
                else:
                    raise RuntimeError(
                        "LAND was accepted but FCU did not disarm within "
                        f"{self.land_disarm_timeout_s:.1f}s"
                    )
        else:
            self._publish_mission_phase("complete_hold")
            self.get_logger().info("Rectangle complete. Holding final setpoint; Ctrl+C to stop.")
            while rclpy.ok():
                self.hold_setpoint(
                    *current, seconds=1.0, yaw=points[-1][3],
                    label="final hold", require_guided=True)


def main(args=None):
    rclpy.init(args=args)
    node = GuidedRectangleWaypoints()
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
