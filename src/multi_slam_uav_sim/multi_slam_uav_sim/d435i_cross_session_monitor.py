
#!/usr/bin/env python3
"""Capture RTAB-Map cross-session candidates, transforms, odometry and GT."""

import csv
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rtabmap_msgs.msg import Info, OdomInfo
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q, vector):
    qx, qy, qz, qw = q
    conjugate = (-qx, -qy, -qz, qw)
    rotated = quaternion_multiply(
        quaternion_multiply(q, (*vector, 0.0)), conjugate)
    return rotated[:3]


def compose(transform, pose):
    translation, tq = transform
    position, pq = pose
    rotated = rotate_vector(tq, position)
    return (
        tuple(translation[index] + rotated[index] for index in range(3)),
        quaternion_multiply(tq, pq),
    )


class D435iCrossSessionMonitor(Node):
    def __init__(self):
        super().__init__("d435i_cross_session_monitor")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("mode", "session")
        self.declare_parameter("condition", "unknown")
        self.declare_parameter(
            "stage_topic", "/d435i_cross_session/stage")
        self.declare_parameter(
            "ground_truth_topic", "/d435i_visual_slam/ground_truth")
        output = str(self.get_parameter("output_dir").value).strip()
        if not output:
            raise ValueError("output_dir is required")
        self.output_dir = Path(output).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.mode = str(self.get_parameter("mode").value)
        self.condition = str(self.get_parameter("condition").value)
        self.started_ns = time.monotonic_ns()
        self.stage = "monitor_started"
        self.latest_gt = None
        self.map_to_odom = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self.last_tf_stamp = {}
        self.tf_backward_jumps = 0

        self.files = []
        self.info_writer = self._csv("info_events.csv", [
            "elapsed_s", "sim_stamp_s", "stage", "ref_id", "candidate_id",
            "candidate_similarity", "candidate_likelihood",
            "candidate_raw_likelihood", "posterior_best", "likelihood_best",
            "raw_likelihood_best", "loop_closure_id",
            "proximity_detection_id", "rejected_hypothesis", "map_id",
            "visual_matches", "visual_inliers", "visual_words",
            "loop_detection_time_ms", "update_time_ms", "optimization_time_ms",
            "closure_x", "closure_y", "closure_z", "closure_yaw_rad",
            "map_to_odom_x", "map_to_odom_y", "map_to_odom_z",
            "map_to_odom_yaw_rad",
        ])
        self.alignment_writer = self._csv("alignment_samples.csv", [
            "elapsed_s", "sim_stamp_s", "stage",
            "odom_x", "odom_y", "odom_z", "odom_yaw_rad",
            "map_x", "map_y", "map_z", "map_yaw_rad",
            "gt_x", "gt_y", "gt_z", "gt_yaw_rad", "gt_age_ms",
            "map_to_odom_x", "map_to_odom_y", "map_to_odom_z",
            "map_to_odom_yaw_rad",
        ])
        self.gt_writer = self._csv("ground_truth.csv", [
            "elapsed_s", "sim_stamp_s", "stage", "x", "y", "z",
            "qx", "qy", "qz", "qw", "yaw_rad",
        ])
        self.odom_writer = self._csv("odometry_health.csv", [
            "elapsed_s", "sim_stamp_s", "stage", "lost", "inliers",
            "features", "matches", "word_inliers", "local_map_size",
            "local_key_frames", "estimation_time_ms",
        ])
        self.stage_writer = self._csv("stage_events.csv", [
            "elapsed_s", "stage", "payload",
        ])
        self.tf_writer = self._csv("tf_events.csv", [
            "elapsed_s", "stamp_s", "parent", "child", "x", "y", "z",
            "yaw_rad", "backward_jump",
        ])

        support = MutuallyExclusiveCallbackGroup()
        info_group = MutuallyExclusiveCallbackGroup()
        odom_group = MutuallyExclusiveCallbackGroup()
        durable = QoSProfile(depth=20)
        durable.reliability = ReliabilityPolicy.RELIABLE
        durable.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, str(self.get_parameter("stage_topic").value),
            self._stage_cb, durable, callback_group=support)
        self.create_subscription(
            Odometry, str(self.get_parameter("ground_truth_topic").value),
            self._gt_cb, qos_profile_sensor_data, callback_group=support)
        self.create_subscription(
            Odometry, "/rtabmap/odom", self._rtab_odom_cb,
            qos_profile_sensor_data, callback_group=odom_group)
        self.create_subscription(
            OdomInfo, "/rtabmap/odom_info", self._odom_info_cb,
            qos_profile_sensor_data, callback_group=odom_group)
        self.create_subscription(
            Info, "/rtabmap/info", self._info_cb, 50,
            callback_group=info_group)
        self.create_subscription(
            TFMessage, "/tf", self._tf_cb, qos_profile_sensor_data,
            callback_group=support)
        (self.output_dir / "monitor_context.json").write_text(json.dumps({
            "mode": self.mode,
            "condition": self.condition,
            "started_wall_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }, indent=2), encoding="utf-8")
        self.get_logger().info(
            f"Cross-session monitor: mode={self.mode} "
            f"condition={self.condition} output={self.output_dir}")

    def _csv(self, name, fields):
        handle = (self.output_dir / name).open(
            "w", newline="", encoding="utf-8", buffering=1)
        self.files.append(handle)
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        return writer

    def elapsed(self):
        return (time.monotonic_ns() - self.started_ns) * 1.0e-9

    @staticmethod
    def _stat(stats, *names):
        for name in names:
            if name in stats:
                return float(stats[name])
        lowered = [(key.lower(), value) for key, value in stats.items()]
        for name in names:
            needle = name.lower().strip("/")
            for key, value in lowered:
                if needle in key:
                    return float(value)
        return 0.0

    def _stage_cb(self, message):
        payload = str(message.data)
        try:
            parsed = json.loads(payload)
            self.stage = str(parsed.get("stage") or "unlabelled")
        except (json.JSONDecodeError, TypeError):
            self.stage = payload or "unlabelled"
        self.stage_writer.writerow({
            "elapsed_s": self.elapsed(), "stage": self.stage,
            "payload": payload,
        })

    def _gt_cb(self, message):
        pose = message.pose.pose
        now_ns = time.monotonic_ns()
        self.latest_gt = (
            now_ns,
            (float(pose.position.x), float(pose.position.y),
             float(pose.position.z)),
            (float(pose.orientation.x), float(pose.orientation.y),
             float(pose.orientation.z), float(pose.orientation.w)),
        )
        self.gt_writer.writerow({
            "elapsed_s": self.elapsed(),
            "sim_stamp_s": stamp_seconds(message.header.stamp),
            "stage": self.stage,
            "x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
            "qx": pose.orientation.x, "qy": pose.orientation.y,
            "qz": pose.orientation.z, "qw": pose.orientation.w,
            "yaw_rad": yaw(pose.orientation),
        })

    def _rtab_odom_cb(self, message):
        pose = message.pose.pose
        odom_pose = (
            (float(pose.position.x), float(pose.position.y),
             float(pose.position.z)),
            (float(pose.orientation.x), float(pose.orientation.y),
             float(pose.orientation.z), float(pose.orientation.w)),
        )
        map_pose = compose(self.map_to_odom, odom_pose)
        gt_position = (float("nan"),) * 3
        gt_yaw = float("nan")
        gt_age_ms = float("nan")
        if self.latest_gt is not None:
            gt_ns, gt_position, gt_q = self.latest_gt
            gt_yaw = math.atan2(
                2.0 * (gt_q[3] * gt_q[2] + gt_q[0] * gt_q[1]),
                1.0 - 2.0 * (gt_q[1] ** 2 + gt_q[2] ** 2))
            gt_age_ms = (time.monotonic_ns() - gt_ns) * 1.0e-6
        mt_position, mt_q = self.map_to_odom
        self.alignment_writer.writerow({
            "elapsed_s": self.elapsed(),
            "sim_stamp_s": stamp_seconds(message.header.stamp),
            "stage": self.stage,
            "odom_x": odom_pose[0][0], "odom_y": odom_pose[0][1],
            "odom_z": odom_pose[0][2],
            "odom_yaw_rad": math.atan2(
                2.0 * (odom_pose[1][3] * odom_pose[1][2]
                       + odom_pose[1][0] * odom_pose[1][1]),
                1.0 - 2.0 * (odom_pose[1][1] ** 2 + odom_pose[1][2] ** 2)),
            "map_x": map_pose[0][0], "map_y": map_pose[0][1],
            "map_z": map_pose[0][2],
            "map_yaw_rad": math.atan2(
                2.0 * (map_pose[1][3] * map_pose[1][2]
                       + map_pose[1][0] * map_pose[1][1]),
                1.0 - 2.0 * (map_pose[1][1] ** 2 + map_pose[1][2] ** 2)),
            "gt_x": gt_position[0], "gt_y": gt_position[1],
            "gt_z": gt_position[2], "gt_yaw_rad": gt_yaw,
            "gt_age_ms": gt_age_ms,
            "map_to_odom_x": mt_position[0],
            "map_to_odom_y": mt_position[1],
            "map_to_odom_z": mt_position[2],
            "map_to_odom_yaw_rad": math.atan2(
                2.0 * (mt_q[3] * mt_q[2] + mt_q[0] * mt_q[1]),
                1.0 - 2.0 * (mt_q[1] ** 2 + mt_q[2] ** 2)),
        })

    def _odom_info_cb(self, message):
        self.odom_writer.writerow({
            "elapsed_s": self.elapsed(),
            "sim_stamp_s": stamp_seconds(message.header.stamp),
            "stage": self.stage, "lost": int(message.lost),
            "inliers": int(message.inliers),
            "features": int(message.features),
            "matches": int(message.matches),
            "word_inliers": len(message.word_inliers),
            "local_map_size": int(message.local_map_size),
            "local_key_frames": int(message.local_key_frames),
            "estimation_time_ms": float(message.time_estimation) * 1000.0,
        })

    def _info_cb(self, message):
        stats = dict(zip(message.stats_keys, message.stats_values))
        likelihood = dict(zip(
            (int(key) for key in message.likelihood_keys),
            (float(value) for value in message.likelihood_values)))
        raw_likelihood = dict(zip(
            (int(key) for key in message.raw_likelihood_keys),
            (float(value) for value in message.raw_likelihood_values)))
        posterior = [
            (int(key), float(value))
            for key, value in zip(message.posterior_keys, message.posterior_values)
            if int(key) > 0 and int(key) != int(message.ref_id)
        ]
        candidate_id = 0
        candidate_similarity = 0.0
        if posterior:
            candidate_id, candidate_similarity = max(
                posterior, key=lambda item: item[1])
        highest_id = self._stat(stats, "Loop/Highest_hypothesis_id/")
        highest_value = self._stat(stats, "Loop/Highest_hypothesis_value/")
        if int(round(highest_id)) > 0:
            candidate_id = int(round(highest_id))
        if highest_value > 0.0:
            candidate_similarity = highest_value
        mt = message.odom_cache.map_to_odom
        self.map_to_odom = (
            (float(mt.translation.x), float(mt.translation.y),
             float(mt.translation.z)),
            (float(mt.rotation.x), float(mt.rotation.y),
             float(mt.rotation.z), float(mt.rotation.w)),
        )
        closure = message.loop_closure_transform
        loop_parts = [self._stat(stats, name) for name in (
            "Timing/Likelihood_computation/ms",
            "Timing/Posterior_computation/ms",
            "Timing/Hypotheses_creation/ms",
            "Timing/Hypotheses_validation/ms",
        )]
        self.info_writer.writerow({
            "elapsed_s": self.elapsed(),
            "sim_stamp_s": stamp_seconds(message.header.stamp),
            "stage": self.stage, "ref_id": int(message.ref_id),
            "candidate_id": candidate_id,
            "candidate_similarity": candidate_similarity,
            "candidate_likelihood": likelihood.get(candidate_id, 0.0),
            "candidate_raw_likelihood": raw_likelihood.get(candidate_id, 0.0),
            "posterior_best": max((value for _, value in posterior), default=0.0),
            "likelihood_best": max(likelihood.values(), default=0.0),
            "raw_likelihood_best": max(raw_likelihood.values(), default=0.0),
            "loop_closure_id": int(message.loop_closure_id),
            "proximity_detection_id": int(message.proximity_detection_id),
            "rejected_hypothesis": self._stat(
                stats, "Loop/RejectedHypothesis/"),
            "map_id": self._stat(stats, "Loop/Map_id/"),
            "visual_matches": self._stat(
                stats, "Loop/Visual_matches/", "Loop/Matches/"),
            "visual_inliers": self._stat(
                stats, "Loop/Visual_inliers/", "Loop/Inliers/"),
            "visual_words": self._stat(stats, "Loop/Visual_words/"),
            "loop_detection_time_ms": sum(loop_parts),
            "update_time_ms": self._stat(stats, "RtabmapROS/TimeTotal/ms"),
            "optimization_time_ms": self._stat(
                stats, "Timing/Map_optimization/ms"),
            "closure_x": closure.translation.x,
            "closure_y": closure.translation.y,
            "closure_z": closure.translation.z,
            "closure_yaw_rad": yaw(closure.rotation),
            "map_to_odom_x": mt.translation.x,
            "map_to_odom_y": mt.translation.y,
            "map_to_odom_z": mt.translation.z,
            "map_to_odom_yaw_rad": yaw(mt.rotation),
        })

    def _tf_cb(self, message):
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            stamp = stamp_seconds(transform.header.stamp)
            backward = int(
                key in self.last_tf_stamp and stamp < self.last_tf_stamp[key])
            self.tf_backward_jumps += backward
            self.last_tf_stamp[key] = stamp
            t = transform.transform
            self.tf_writer.writerow({
                "elapsed_s": self.elapsed(), "stamp_s": stamp,
                "parent": key[0], "child": key[1],
                "x": t.translation.x, "y": t.translation.y,
                "z": t.translation.z, "yaw_rad": yaw(t.rotation),
                "backward_jump": backward,
            })

    def close(self):
        (self.output_dir / "monitor_final.json").write_text(json.dumps({
            "elapsed_s": self.elapsed(),
            "last_stage": self.stage,
            "tf_backward_jumps": self.tf_backward_jumps,
        }, indent=2), encoding="utf-8")
        for handle in self.files:
            handle.close()
        self.files.clear()


def main(args=None):
    rclpy.init(args=args)
    node = D435iCrossSessionMonitor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

