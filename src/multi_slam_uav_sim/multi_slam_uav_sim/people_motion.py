import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import json, math, time
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from gz.transport13 import Node as GzNode
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V

Point = Tuple[float, float, float]


@dataclass
class Agent:
    name: str
    path_type: str
    points: List[Point]
    speed: float
    start_delay: float = 0.0
    distance_offset: float = 0.0


def _segment_pose(points: Sequence[Point], distance: float, closed: bool):
    segments, total = [], 0.0
    limit = len(points) if closed else len(points) - 1
    for i in range(limit):
        a, b = points[i], points[(i + 1) % len(points)]
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length > 1e-6:
            segments.append((a, b, length))
            total += length
    if total <= 1e-6:
        return points[0], 0.0
    d = distance % total
    for a, b, length in segments:
        if d <= length:
            ratio = d / length
            pos = (
                a[0] + (b[0] - a[0]) * ratio,
                a[1] + (b[1] - a[1]) * ratio,
                a[2] + (b[2] - a[2]) * ratio,
            )
            return pos, math.atan2(b[1] - a[1], b[0] - a[0])
        d -= length
    a, b, _ = segments[-1]
    return b, math.atan2(b[1] - a[1], b[0] - a[0])


def _line_pingpong(points: Sequence[Point], distance: float):
    a, b = points[0], points[1]
    length = max(math.hypot(b[0] - a[0], b[1] - a[1]), 1e-6)
    phase = (distance % (2.0 * length)) / length
    forward = phase <= 1.0
    ratio = phase if forward else 2.0 - phase
    pos = (
        a[0] + (b[0] - a[0]) * ratio,
        a[1] + (b[1] - a[1]) * ratio,
        a[2] + (b[2] - a[2]) * ratio,
    )
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    if not forward:
        yaw = (yaw + math.pi + math.pi) % (2.0 * math.pi) - math.pi
    return pos, yaw


def _two_lane_line(points: Sequence[Point], distance: float):
    """Two one-way lanes with teleport at each road end.

    points = [out_start, out_end, return_start, return_end].
    The actor drives out_start -> out_end, instantly moves to return_start,
    drives return_start -> return_end, then instantly moves back to out_start.
    """

    if len(points) < 4:
        return _line_pingpong(points, distance)

    a, b, c, d = points[:4]
    len_out = max(math.hypot(b[0] - a[0], b[1] - a[1]), 1e-6)
    len_back = max(math.hypot(d[0] - c[0], d[1] - c[1]), 1e-6)
    phase = distance % (len_out + len_back)

    if phase <= len_out:
        ratio = phase / len_out
        pos = (
            a[0] + (b[0] - a[0]) * ratio,
            a[1] + (b[1] - a[1]) * ratio,
            a[2] + (b[2] - a[2]) * ratio,
        )
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        return pos, yaw

    ratio = (phase - len_out) / len_back
    pos = (
        c[0] + (d[0] - c[0]) * ratio,
        c[1] + (d[1] - c[1]) * ratio,
        c[2] + (d[2] - c[2]) * ratio,
    )
    yaw = math.atan2(d[1] - c[1], d[0] - c[0])
    return pos, yaw


def _fill_pose(msg, name: str, pos: Point, yaw: float):
    half = yaw * 0.5
    msg.name = name
    msg.position.x = float(pos[0])
    msg.position.y = float(pos[1])
    msg.position.z = float(pos[2])
    msg.orientation.x = 0.0
    msg.orientation.y = 0.0
    msg.orientation.z = math.sin(half)
    msg.orientation.w = math.cos(half)


class PeopleMotion(Node):
    def __init__(self):
        super().__init__('people_motion')
        self.declare_parameter('world_name', 'basic_people')
        self.declare_parameter('agents_json', '[]')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('gz_timeout_ms', 100)
        self.world_name = self.get_parameter('world_name').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.gz_timeout_ms = int(self.get_parameter('gz_timeout_ms').value)
        self.agents = self._load_agents(self.get_parameter('agents_json').value)
        self.gz_node = GzNode()
        self.start_time, self.warned = time.monotonic(), False
        self.failed_requests = 0
        self.get_logger().info(f'Controlling {len(self.agents)} visual-only actors in [{self.world_name}]')
        for agent in self.agents:
            self.get_logger().info(
                f'{agent.name}: {agent.path_type}, speed={agent.speed}, '
                f'start_delay={agent.start_delay}, distance_offset={agent.distance_offset}, points={agent.points}'
            )
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self._tick)

    def _load_agents(self, raw):
        agents = []
        for item in json.loads(raw):
            agents.append(Agent(
                item['name'],
                item.get('path_type', 'line'),
                [tuple(float(v) for v in point) for point in item['points']],
                float(item.get('speed', 0.8)),
                float(item.get('start_delay', 0.0)),
                float(item.get('distance_offset', 0.0)),
            ))
        return agents

    def _tick(self):
        now = time.monotonic() - self.start_time
        req = Pose_V()
        for agent in self.agents:
            elapsed = max(0.0, now - agent.start_delay)
            distance = agent.distance_offset + elapsed * agent.speed
            if agent.path_type == 'line':
                pos, yaw = _line_pingpong(agent.points, distance)
            elif agent.path_type == 'two_lane_line':
                pos, yaw = _two_lane_line(agent.points, distance)
            else:
                pos, yaw = _segment_pose(agent.points, distance, True)
            _fill_pose(req.pose.add(), agent.name, pos, yaw)
        self._set_pose_vector(req)

    def _set_pose_vector(self, req: Pose_V):
        try:
            ok, reply = self.gz_node.request(
                f'/world/{self.world_name}/set_pose_vector',
                req,
                Pose_V,
                Boolean,
                self.gz_timeout_ms,
            )
        except Exception as exc:
            if not self.warned:
                self.get_logger().warning(f'Failed to call Gazebo transport set_pose_vector service: {exc}')
                self.warned = True
            return
        if ok and getattr(reply, 'data', False):
            self.failed_requests = 0
            return
        self.failed_requests += 1
        if self.failed_requests > max(10, int(self.rate_hz * 3.0)) and not self.warned:
            self.get_logger().warning('Gazebo set_pose_vector service is not ready or returned false. Start gz sim first or wait a few seconds.')
            self.warned = True


def main(args=None):
    rclpy.init(args=args)
    node = PeopleMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
