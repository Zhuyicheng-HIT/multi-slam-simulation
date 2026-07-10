import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node


class GuidedFlightDemo(Node):
    def __init__(self):
        super().__init__('guided_flight_demo')
        self.declare_parameter('takeoff_alt', 4.0)
        self.declare_parameter('side_length', 4.0)
        self.declare_parameter('hold_time', 6.0)
        self.declare_parameter('setpoint_rate_hz', 10.0)
        self.declare_parameter('land_at_end', False)

        self.takeoff_alt = float(self.get_parameter('takeoff_alt').value)
        self.side_length = float(self.get_parameter('side_length').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.land_at_end = bool(self.get_parameter('land_at_end').value)

        self.state = State()
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_cli = self.create_client(CommandTOL, '/mavros/cmd/land')

    def _state_cb(self, msg):
        self.state = msg

    def wait_ready(self, timeout=60.0):
        self.get_logger().info('Waiting for MAVROS FCU connection...')
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state.connected:
                self.get_logger().info('MAVROS connected to FCU')
                break
        else:
            raise RuntimeError('Timed out waiting for /mavros/state.connected')
        for client, name in [
            (self.arming_cli, 'arming'),
            (self.mode_cli, 'set_mode'),
            (self.takeoff_cli, 'takeoff'),
        ]:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f'MAVROS service not available: {name}')

    def call(self, client, request, label, timeout=10.0):
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                resp = future.result()
                self.get_logger().info(f'{label}: {resp}')
                return resp
        raise RuntimeError(f'Timeout calling {label}')

    def publish_setpoint(self, x, y, z, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.setpoint_pub.publish(msg)

    def hold_setpoint(self, x, y, z, seconds, yaw=0.0):
        period = 1.0 / max(self.rate_hz, 1.0)
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.publish_setpoint(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def run_demo(self):
        self.wait_ready()
        self.get_logger().info('Priming local position setpoints...')
        self.hold_setpoint(0.0, 0.0, self.takeoff_alt, 2.0)

        mode_req = SetMode.Request()
        mode_req.base_mode = 0
        mode_req.custom_mode = 'GUIDED'
        self.call(self.mode_cli, mode_req, 'set GUIDED')

        arm_req = CommandBool.Request()
        arm_req.value = True
        self.call(self.arming_cli, arm_req, 'arm')

        takeoff_req = CommandTOL.Request()
        takeoff_req.min_pitch = 0.0
        takeoff_req.yaw = 0.0
        takeoff_req.latitude = 0.0
        takeoff_req.longitude = 0.0
        takeoff_req.altitude = self.takeoff_alt
        self.call(self.takeoff_cli, takeoff_req, f'takeoff {self.takeoff_alt:.1f}m')

        self.get_logger().info('Climbing and holding initial point...')
        self.hold_setpoint(0.0, 0.0, self.takeoff_alt, 8.0)

        points = [
            (0.0, 0.0, self.takeoff_alt, 0.0),
            (self.side_length, 0.0, self.takeoff_alt, 0.0),
            (self.side_length, self.side_length, self.takeoff_alt, 1.57),
            (0.0, self.side_length, self.takeoff_alt, 3.14),
            (0.0, 0.0, self.takeoff_alt, -1.57),
        ]
        for i, (x, y, z, yaw) in enumerate(points, start=1):
            self.get_logger().info(f'GUIDED setpoint {i}/{len(points)}: x={x:.1f}, y={y:.1f}, z={z:.1f}')
            self.hold_setpoint(x, y, z, self.hold_time, yaw)

        if self.land_at_end:
            if self.land_cli.wait_for_service(timeout_sec=5.0):
                land_req = CommandTOL.Request()
                land_req.min_pitch = 0.0
                land_req.yaw = 0.0
                land_req.latitude = 0.0
                land_req.longitude = 0.0
                land_req.altitude = 0.0
                self.call(self.land_cli, land_req, 'land')
        else:
            self.get_logger().info('Demo complete. Holding final GUIDED setpoint; Ctrl+C to stop.')
            while rclpy.ok():
                self.hold_setpoint(0.0, 0.0, self.takeoff_alt, 1.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = GuidedFlightDemo()
    try:
        node.run_demo()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
