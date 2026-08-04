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
        self.declare_parameter("response_wait_s", 3.0)

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

    def _call(self, client, request, label):
        """Send one MAVLink rate command and wait for its ACK before the next."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=max(0.1, self.response_wait_s)
        )
        if not future.done():
            future.cancel()
            self.get_logger().warning(f"No service response before deadline: {label}")
            return False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(f"{label} failed: {exc}")
            return False
        self.get_logger().info(f"{label}: {result}")
        return bool(getattr(result, "success", True))

    def _request_streams(self):
        stream_ids = [
            StreamRate.Request.STREAM_RAW_SENSORS,
            StreamRate.Request.STREAM_EXTENDED_STATUS,
            StreamRate.Request.STREAM_POSITION,
            StreamRate.Request.STREAM_EXTRA1,
            StreamRate.Request.STREAM_EXTRA2,
            StreamRate.Request.STREAM_EXTRA3,
        ]
        # MAVLink common message IDs used by MAVROS local/global/IMU plugins.
        # HIGHRES_IMU is first because FAST-LIO cannot start without it.
        intervals = [
            (105, float(self.get_parameter("imu_rate_hz").value)),    # HIGHRES_IMU
            (24, float(self.get_parameter("gps_rate_hz").value)),     # GPS_RAW_INT
            (30, float(self.get_parameter("attitude_rate_hz").value)),  # ATTITUDE
            (31, float(self.get_parameter("attitude_rate_hz").value)),  # ATTITUDE_QUATERNION
            (32, float(self.get_parameter("position_rate_hz").value)),  # LOCAL_POSITION_NED
            (33, float(self.get_parameter("position_rate_hz").value)),  # GLOBAL_POSITION_INT
            (74, 10.0),                                                   # VFR_HUD
            (193, 2.0),                                                   # EKF_STATUS_REPORT
            # Use one FCU source. HIGHRES_IMU feeds MAVROS data_raw directly;
            # disabling RAW_IMU prevents multiple raw sensor sources.
            (27, 0.0),                                                    # RAW_IMU off
        ]
        highres_imu_ok = False
        if self._wait_service(self.interval_cli, "set_message_interval"):
            for msg_id, rate_hz in intervals:
                req = MessageInterval.Request()
                req.message_id = int(msg_id)
                req.message_rate = float(rate_hz)
                accepted = self._call(
                    self.interval_cli,
                    req,
                    f"message_interval id={msg_id} hz={rate_hz}",
                )
                if msg_id == 105:
                    highres_imu_ok = accepted

        if self._wait_service(self.stream_cli, "set_stream_rate"):
            for stream_id in stream_ids:
                if highres_imu_ok and stream_id == StreamRate.Request.STREAM_RAW_SENSORS:
                    continue
                req = StreamRate.Request()
                req.stream_id = stream_id
                req.message_rate = self.stream_rate_hz
                req.on_off = True
                self._call(
                    self.stream_cli,
                    req,
                    f"stream_rate id={stream_id} hz={self.stream_rate_hz}",
                )

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
