import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval, StreamRate
from rclpy.node import Node


class MavrosStreamRequester(Node):
    """Request ArduPilot telemetry streams needed by companion nodes."""

    def __init__(self):
        super().__init__("mavros_stream_requester")
        self.declare_parameter("mavros_ns", "/mavros")
        self.declare_parameter("stream_rate_hz", 20)
        self.declare_parameter("position_rate_hz", 20.0)
        self.declare_parameter("imu_rate_hz", 100.0)
        self.declare_parameter("gps_rate_hz", 10.0)
        self.declare_parameter("attitude_rate_hz", 30.0)
        self.declare_parameter("timeout_s", 30.0)
        self.declare_parameter("response_wait_s", 2.0)

        self.mavros_ns = str(self.get_parameter("mavros_ns").value).rstrip("/")
        self.stream_rate_hz = int(self.get_parameter("stream_rate_hz").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.response_wait_s = float(self.get_parameter("response_wait_s").value)
        self.connected = False
        self.start_time = time.monotonic()

        self.create_subscription(State, f"{self.mavros_ns}/state", self._state_cb, 10)
        self.stream_cli = self.create_client(StreamRate, f"{self.mavros_ns}/set_stream_rate")
        self.interval_cli = self.create_client(MessageInterval, f"{self.mavros_ns}/set_message_interval")

    def _state_cb(self, msg):
        self.connected = bool(msg.connected)

    def wait_connected(self):
        end = time.monotonic() + self.timeout_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.connected:
                self.get_logger().info("MAVROS connected; requesting ArduPilot telemetry streams...")
                return True
        self.get_logger().warning("Timed out waiting for MAVROS FCU connection")
        return False

    def _wait_service(self, client, label, timeout=3.0):
        if client.wait_for_service(timeout_sec=timeout):
            return True
        self.get_logger().warning(f"Service unavailable, skipping {label}")
        return False

    def _collect(self, futures):
        end = time.monotonic() + max(0.1, self.response_wait_s)
        pending = dict(futures)
        while rclpy.ok() and pending and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            for future, label in list(pending.items()):
                if future.done():
                    try:
                        self.get_logger().info(f"{label}: {future.result()}")
                    except Exception as exc:
                        self.get_logger().warning(f"{label} failed: {exc}")
                    pending.pop(future, None)
        for label in pending.values():
            self.get_logger().warning(f"No service response before deadline: {label}")

    def _request_streams(self):
        futures = {}
        stream_ids = [
            StreamRate.Request.STREAM_RAW_SENSORS,
            StreamRate.Request.STREAM_EXTENDED_STATUS,
            StreamRate.Request.STREAM_POSITION,
            StreamRate.Request.STREAM_EXTRA1,
            StreamRate.Request.STREAM_EXTRA2,
            StreamRate.Request.STREAM_EXTRA3,
        ]
        if self._wait_service(self.stream_cli, "set_stream_rate"):
            for stream_id in stream_ids:
                req = StreamRate.Request()
                req.stream_id = stream_id
                req.message_rate = self.stream_rate_hz
                req.on_off = True
                futures[self.stream_cli.call_async(req)] = (
                    f"stream_rate id={stream_id} hz={self.stream_rate_hz}"
                )

        # MAVLink common message IDs used by MAVROS local/global/IMU plugins.
        intervals = {
            24: float(self.get_parameter("gps_rate_hz").value),       # GPS_RAW_INT
            30: float(self.get_parameter("attitude_rate_hz").value),  # ATTITUDE
            31: float(self.get_parameter("attitude_rate_hz").value),  # ATTITUDE_QUATERNION
            32: float(self.get_parameter("position_rate_hz").value),  # LOCAL_POSITION_NED
            33: float(self.get_parameter("position_rate_hz").value),  # GLOBAL_POSITION_INT
            74: 10.0,                                                  # VFR_HUD
            # Use one FCU source. HIGHRES_IMU feeds MAVROS data_raw directly;
            # disabling RAW_IMU prevents multiple raw sensor sources.
            27: 0.0,                                                     # RAW_IMU off
            105: float(self.get_parameter("imu_rate_hz").value),       # HIGHRES_IMU
        }
        if self._wait_service(self.interval_cli, "set_message_interval"):
            for msg_id, rate_hz in intervals.items():
                req = MessageInterval.Request()
                req.message_id = int(msg_id)
                req.message_rate = float(rate_hz)
                futures[self.interval_cli.call_async(req)] = (
                    f"message_interval id={msg_id} hz={rate_hz}"
                )
        self._collect(futures)

    def run(self):
        if not self.wait_connected():
            return False
        self._request_streams()
        self.get_logger().info("ArduPilot telemetry stream request complete.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MavrosStreamRequester()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
