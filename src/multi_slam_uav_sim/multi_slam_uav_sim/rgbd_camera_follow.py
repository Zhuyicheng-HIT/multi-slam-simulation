import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node as RosNode
from gz.transport13 import Node as GzNode
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vec(q, v):
    x, y, z, w = q
    qv = (v[0], v[1], v[2], 0.0)
    qi = (-x, -y, -z, w)
    r = quat_multiply(quat_multiply(q, qv), qi)
    return (r[0], r[1], r[2])


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class RgbdCameraFollow(RosNode):
    def __init__(self):
        super().__init__('rgbd_camera_follow')
        self.declare_parameter('world_name', 'city_apm_rgbd')
        self.declare_parameter('target_model', 'apm_iris')
        self.declare_parameter('camera_model', 'apm_iris_d435i')
        self.declare_parameter('offset_xyz', [0.16, 0.0, -0.08])
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('timeout_ms', 1000)
        self.declare_parameter('min_world_z', 0.35)
        self.declare_parameter('follow_attitude', True)
        self.declare_parameter('camera_rpy', [0.0, 0.0, 0.0])

        self.world_name = self.get_parameter('world_name').value
        self.target_model = self.get_parameter('target_model').value
        self.camera_model = self.get_parameter('camera_model').value
        self.offset_xyz = tuple(float(x) for x in self.get_parameter('offset_xyz').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.min_period = 1.0 / max(self.rate_hz, 1.0)
        self.timeout_ms = int(self.get_parameter('timeout_ms').value)
        self.min_world_z = float(self.get_parameter('min_world_z').value)
        self.follow_attitude = bool(self.get_parameter('follow_attitude').value)
        self.camera_rpy = tuple(float(x) for x in self.get_parameter('camera_rpy').value)
        self.camera_q = quat_from_rpy(*self.camera_rpy)
        self.gz_node = GzNode()
        self.last_target_pose = None
        self.last_seen = 0.0
        self.last_sent = 0.0
        self.last_report = 0.0
        self.warned = False

        self.gz_node.subscribe(Pose_V, f'/world/{self.world_name}/pose/info', self._pose_cb)
        self.gz_node.subscribe(Pose_V, f'/world/{self.world_name}/dynamic_pose/info', self._pose_cb)
        self.timer = self.create_timer(self.min_period, self._tick)
        self.get_logger().info(f'Following Gazebo model [{self.target_model}] with RGB-D rig [{self.camera_model}] in world [{self.world_name}]')

    def _pose_cb(self, msg: Pose_V):
        for pose in msg.pose:
            if pose.name == self.target_model or pose.name.endswith(f'::{self.target_model}'):
                self.last_target_pose = pose
                self.last_seen = time.monotonic()
                self._tick()
                return

    def _tick(self):
        pose = self.last_target_pose
        if pose is None:
            if not self.warned:
                self.get_logger().warning(f'Waiting for Gazebo pose of [{self.target_model}]')
                self.warned = True
            return

        now = time.monotonic()
        if now - self.last_sent < self.min_period:
            return
        self.last_sent = now

        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        ox, oy, oz = rotate_vec(q, self.offset_xyz)
        req = Pose()
        req.name = self.camera_model
        req.position.x = pose.position.x + ox
        req.position.y = pose.position.y + oy
        req.position.z = max(pose.position.z + oz, self.min_world_z)
        if self.follow_attitude:
            cq = quat_multiply(q, self.camera_q)
            req.orientation.x = cq[0]
            req.orientation.y = cq[1]
            req.orientation.z = cq[2]
            req.orientation.w = cq[3]
        else:
            req.orientation.x = 0.0
            req.orientation.y = 0.0
            req.orientation.z = 0.0
            req.orientation.w = 1.0
        try:
            ok, reply = self.gz_node.request(
                f'/world/{self.world_name}/set_pose',
                req,
                Pose,
                Boolean,
                self.timeout_ms,
            )
        except Exception as exc:
            if not self.warned:
                self.get_logger().warning(f'Failed to move RGB-D camera model: {exc}')
                self.warned = True
            return
        if ok and getattr(reply, 'data', False):
            self.warned = False
            if now - self.last_report > 3.0:
                self.get_logger().info(
                    f'follow ok: {self.camera_model} -> '
                    f'({req.position.x:.2f}, {req.position.y:.2f}, {req.position.z:.2f}) '
                    f'from {self.target_model}=({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})'
                )
                self.last_report = now
        elif now - self.last_report > 3.0:
            self.get_logger().warning(
                f'Gazebo set_pose rejected for [{self.camera_model}] in world [{self.world_name}]: '
                f'ok={ok}, reply={getattr(reply, "data", None)}'
            )
            self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = RgbdCameraFollow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
