#!/usr/bin/env python3
import bisect
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from multi_slam_uav_sim.rtabmap_robustness_profiler import (
    RtabmapRobustnessProfiler,
    finite_summary,
    stamp_ns,
    stamp_seconds,
    wrap_angle,
    yaw_from_quaternion,
)


def quaternion_rpy(quaternion):
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = yaw_from_quaternion(x, y, z, w)
    return roll, pitch, yaw


def write_csv(path, fields, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row.get(key, "") for key in fields} for row in rows)


def longest_true_duration(rows, predicate):
    start = None
    longest = 0.0
    previous = None
    for row in rows:
        stamp = float(row["stamp"])
        if predicate(row):
            if start is None or (previous is not None and stamp - previous > 0.25):
                start = stamp
            longest = max(longest, stamp - start)
        else:
            start = None
        previous = stamp
    return longest


class D435iSpeedEnvelopeProfiler(RtabmapRobustnessProfiler):
    def __init__(self):
        self.commanded_motion = []
        self.motion_records = {"ground_truth": [], "mavros": []}
        self.last_motion = {}
        self.imu_records = []
        super().__init__()
        self.declare_parameter("speed_test_profile", "horizontal")
        self.declare_parameter("commanded_horizontal_speed_mps", 0.0)
        self.declare_parameter("commanded_vertical_speed_mps", 0.0)
        self.declare_parameter("commanded_yaw_rate_deg_s", 0.0)
        self.declare_parameter(
            "commanded_motion_topic", "/d435i_visual_slam/commanded_motion")
        self.speed_test_profile = str(
            self.get_parameter("speed_test_profile").value)
        self.target_horizontal = max(0.0, float(
            self.get_parameter("commanded_horizontal_speed_mps").value))
        self.target_vertical = max(0.0, float(
            self.get_parameter("commanded_vertical_speed_mps").value))
        self.target_yaw_deg_s = max(0.0, float(
            self.get_parameter("commanded_yaw_rate_deg_s").value))
        self.speed_callbacks = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("commanded_motion_topic").value),
            self._commanded_cb, 50, callback_group=self.speed_callbacks)
        self.create_subscription(
            Imu, "/mavros/imu/data", self._imu_cb,
            qos_profile_sensor_data, callback_group=self.speed_callbacks)
        self.get_logger().info(
            f"Speed profiler: profile={self.speed_test_profile} "
            f"horizontal={self.target_horizontal:.3f}m/s "
            f"vertical={self.target_vertical:.3f}m/s "
            f"yaw={self.target_yaw_deg_s:.1f}deg/s")

    def _commanded_cb(self, message):
        linear = message.twist.linear
        angular = message.twist.angular
        self.commanded_motion.append({
            "stamp": stamp_seconds(message.header.stamp),
            "arrival_steady_s": time.monotonic_ns() * 1.0e-9,
            "stage": self.stage,
            "vx": float(linear.x), "vy": float(linear.y),
            "vz": float(linear.z),
            "horizontal_speed": math.hypot(linear.x, linear.y),
            "vertical_speed": float(linear.z),
            "yaw_rate_rad_s": float(angular.z),
            "yaw_rate_deg_s": math.degrees(float(angular.z)),
        })

    def _imu_cb(self, message):
        roll, pitch, yaw = quaternion_rpy(message.orientation)
        self.imu_records.append({
            "stamp": stamp_seconds(message.header.stamp),
            "arrival_steady_s": time.monotonic_ns() * 1.0e-9,
            "stage": self.stage,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "linear_accel_x": float(message.linear_acceleration.x),
            "linear_accel_y": float(message.linear_acceleration.y),
            "linear_accel_z": float(message.linear_acceleration.z),
            "angular_x_deg_s": math.degrees(message.angular_velocity.x),
            "angular_y_deg_s": math.degrees(message.angular_velocity.y),
            "angular_z_deg_s": math.degrees(message.angular_velocity.z),
        })

    def _odom_info_cb(self, message):
        super()._odom_info_cb(message)
        event = self.odom_events[-1]
        event["words"] = len(message.words_keys)
        event["word_matches"] = len(message.word_matches)
        event["word_inliers"] = len(message.word_inliers)
        event["inlier_ratio"] = (
            float(message.inliers) / float(message.matches)
            if message.matches else 0.0)
        diagonal = [float(message.covariance[index]) for index in (0, 7, 14)]
        event["covariance_translation_trace"] = sum(diagonal)
        event["covariance_rotation_trace"] = sum(
            float(message.covariance[index]) for index in (21, 28, 35))

    def _odom_cb(self, stream, message):
        super()._odom_cb(stream, message)
        if stream not in self.motion_records:
            return
        raw_stamp = stamp_seconds(message.header.stamp)
        evaluation_stamp = raw_stamp
        if stream == "mavros" and self.clock_samples:
            evaluation_stamp = self.clock_samples[-1][1]
        pose = message.pose.pose
        roll, pitch, yaw = quaternion_rpy(pose.orientation)
        position = np.asarray([
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z)], dtype=float)
        reported_linear = message.twist.twist.linear
        reported_angular = message.twist.twist.angular
        previous = self.last_motion.get(stream)
        derived_velocity = np.zeros(3)
        derived_acceleration = np.zeros(3)
        derived_yaw_rate = 0.0
        if previous is not None:
            delta_s = evaluation_stamp - previous["stamp"]
            if 1.0e-4 < delta_s < 0.5:
                derived_velocity = (position - previous["position"]) / delta_s
                derived_acceleration = (
                    derived_velocity - previous["derived_velocity"]) / delta_s
                derived_yaw_rate = wrap_angle(yaw - previous["yaw"]) / delta_s
        record = {
            "stamp": evaluation_stamp,
            "raw_header_stamp": raw_stamp,
            "arrival_steady_s": time.monotonic_ns() * 1.0e-9,
            "stage": self.stage,
            "x": position[0], "y": position[1], "z": position[2],
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "reported_vx": float(reported_linear.x),
            "reported_vy": float(reported_linear.y),
            "reported_vz": float(reported_linear.z),
            "reported_horizontal_speed": math.hypot(
                reported_linear.x, reported_linear.y),
            "reported_yaw_rate_deg_s": math.degrees(reported_angular.z),
            "derived_vx": derived_velocity[0],
            "derived_vy": derived_velocity[1],
            "derived_vz": derived_velocity[2],
            "derived_horizontal_speed": math.hypot(
                derived_velocity[0], derived_velocity[1]),
            "derived_yaw_rate_deg_s": math.degrees(derived_yaw_rate),
            "derived_accel_x": derived_acceleration[0],
            "derived_accel_y": derived_acceleration[1],
            "derived_accel_z": derived_acceleration[2],
            "derived_yaw_accel_deg_s2": 0.0,
        }
        self.motion_records[stream].append(record)
        self.last_motion[stream] = {
            "stamp": evaluation_stamp, "position": position,
            "yaw": yaw, "derived_velocity": derived_velocity,
        }

    def _active_stage(self, stage):
        marker = f"speed_{self.speed_test_profile}:"
        if not stage.startswith(marker):
            return False
        label = stage[len(marker):]
        selectors = {
            "horizontal": ("horizontal_out", "horizontal_return"),
            "vertical": ("vertical_up", "vertical_down"),
            "yaw": ("yaw_out", "yaw_return"),
            "combined": ("combined_out", "combined_return"),
            "small_rectangle": ("rectangle_leg_",),
            "loop_return": ("loop_leg_", "loop_return_to_start"),
        }
        return label.startswith(selectors.get(self.speed_test_profile, ()))

    @staticmethod
    def _nearest(rows, stamps, stamp, tolerance=0.15):
        if not rows:
            return None
        index = bisect.bisect_left(stamps, stamp)
        candidates = [value for value in (index - 1, index)
                      if 0 <= value < len(rows)]
        if not candidates:
            return None
        best = min(candidates, key=lambda value: abs(stamps[value] - stamp))
        return rows[best] if abs(stamps[best] - stamp) <= tolerance else None

    def _associate_commands(self, rows):
        commands = sorted(self.commanded_motion, key=lambda row: row["stamp"])
        stamps = [row["stamp"] for row in commands]
        for row in rows:
            command = self._nearest(commands, stamps, row["stamp"], 0.25)
            if command:
                row["commanded_horizontal_speed"] = command["horizontal_speed"]
                row["commanded_vertical_speed"] = command["vertical_speed"]
                row["commanded_yaw_rate_deg_s"] = command["yaw_rate_deg_s"]
            else:
                row["commanded_horizontal_speed"] = 0.0
                row["commanded_vertical_speed"] = 0.0
                row["commanded_yaw_rate_deg_s"] = 0.0
            actual, target, command_value = self._metric_values(row)
            if abs(command_value) > 0.01:
                row["phase"] = "steady" if actual >= 0.8 * target else "acceleration"
            elif actual >= 0.1 * target:
                row["phase"] = "deceleration"
            else:
                row["phase"] = "hold"

    @staticmethod
    def _smooth_motion(rows, window_s=0.45):
        """Derive velocity over a finite window so repeated Gazebo poses do not
        look like alternating zero/double-speed samples."""
        for index, row in enumerate(rows):
            earlier = index - 1
            while earlier > 0 and row["stamp"] - rows[earlier]["stamp"] < window_s:
                earlier -= 1
            delta_s = row["stamp"] - rows[earlier]["stamp"] if index else 0.0
            if not (0.15 <= delta_s <= 1.0):
                continue
            previous = rows[earlier]
            vx = (row["x"] - previous["x"]) / delta_s
            vy = (row["y"] - previous["y"]) / delta_s
            vz = (row["z"] - previous["z"]) / delta_s
            yaw_rate = wrap_angle(math.radians(
                row["yaw_deg"] - previous["yaw_deg"])) / delta_s
            row["derived_vx"] = vx
            row["derived_vy"] = vy
            row["derived_vz"] = vz
            row["derived_horizontal_speed"] = math.hypot(vx, vy)
            row["derived_yaw_rate_deg_s"] = math.degrees(yaw_rate)
        for current, previous in zip(rows[1:], rows[:-1]):
            delta_s = current["stamp"] - previous["stamp"]
            if not (1.0e-4 < delta_s < 0.5):
                continue
            current["derived_accel_x"] = (
                current["derived_vx"] - previous["derived_vx"]) / delta_s
            current["derived_accel_y"] = (
                current["derived_vy"] - previous["derived_vy"]) / delta_s
            current["derived_accel_z"] = (
                current["derived_vz"] - previous["derived_vz"]) / delta_s
            current["derived_yaw_accel_deg_s2"] = (
                current["derived_yaw_rate_deg_s"]
                - previous["derived_yaw_rate_deg_s"]) / delta_s

    def _metric_values(self, row):
        if self.speed_test_profile == "yaw":
            return (
                abs(float(row["derived_yaw_rate_deg_s"])),
                self.target_yaw_deg_s,
                abs(float(row.get("commanded_yaw_rate_deg_s", 0.0))),
            )
        if self.speed_test_profile == "vertical":
            return (
                abs(float(row["derived_vz"])), self.target_vertical,
                abs(float(row.get("commanded_vertical_speed", 0.0))),
            )
        return (
            abs(float(row["derived_horizontal_speed"])), self.target_horizontal,
            abs(float(row.get("commanded_horizontal_speed", 0.0))),
        )

    def _motion_summary(self, ground_truth):
        active = [row for row in ground_truth if self._active_stage(row["stage"])]
        values = [self._metric_values(row)[0] for row in active]
        target = (self.target_yaw_deg_s if self.speed_test_profile == "yaw"
                  else self.target_vertical if self.speed_test_profile == "vertical"
                  else self.target_horizontal)
        sustained = longest_true_duration(
            active,
            lambda row: (
                self._metric_values(row)[0] >= 0.8 * target
                and self._metric_values(row)[2] >= 0.8 * target),
        ) if target > 0.0 else 0.0
        summary = finite_summary(values) or {}
        summary.update({
            "target": target,
            "sustained_above_80_percent_s": sustained,
            "valid_speed_experiment": sustained >= 1.0,
            "active_samples": len(active),
        })
        for phase in ("acceleration", "steady", "deceleration"):
            summary[f"{phase}_samples"] = sum(
                row.get("phase") == phase for row in active)
        if self.speed_test_profile == "combined":
            simultaneous = longest_true_duration(
                active,
                lambda row: (
                    float(row["derived_horizontal_speed"]) >= 0.8 * self.target_horizontal
                    and abs(float(row["derived_yaw_rate_deg_s"]))
                    >= 0.8 * self.target_yaw_deg_s),
            )
            summary["simultaneous_above_80_percent_s"] = simultaneous
            summary["valid_speed_experiment"] = simultaneous >= 1.0
        return summary

    def _processed_frame_rows(self):
        ground_truth = sorted(
            self.motion_records["ground_truth"], key=lambda row: row["stamp"])
        gt_stamps = [row["stamp"] for row in ground_truth]
        event_by_stamp = {event["input_stamp_ns"]: event
                          for event in self.odom_events}
        rtab_samples = sorted(self.trajectories["rtabmap"], key=lambda row: row[0])
        rows = []
        previous = None
        for sample in rtab_samples:
            stamp = float(sample[0])
            gt = self._nearest(ground_truth, gt_stamps, stamp)
            arrival = self.odom_outputs.get(int(round(stamp * 1.0e9)))
            event = event_by_stamp.get(int(round(stamp * 1.0e9)), {})
            if gt is None or arrival is None:
                continue
            current = {
                "stamp": stamp,
                "arrival_steady_ns": arrival,
                "gt": gt,
                "estimate": sample,
                "event": event,
            }
            if previous is not None:
                gt_delta = math.dist(
                    [gt[axis] for axis in ("x", "y", "z")],
                    [previous["gt"][axis] for axis in ("x", "y", "z")])
                estimate_delta = math.dist(sample[1:4], previous["estimate"][1:4])
                gt_yaw_delta = abs(wrap_angle(math.radians(
                    gt["yaw_deg"] - previous["gt"]["yaw_deg"])))
                estimate_yaw = yaw_from_quaternion(*sample[4:8])
                previous_yaw = yaw_from_quaternion(*previous["estimate"][4:8])
                estimate_yaw_delta = abs(wrap_angle(estimate_yaw - previous_yaw))
                image_delta = stamp - previous["stamp"]
                wall_delta = (arrival - previous["arrival_steady_ns"]) * 1.0e-9
            else:
                gt_delta = estimate_delta = gt_yaw_delta = estimate_yaw_delta = 0.0
                image_delta = wall_delta = 0.0
            matches = event.get("matches")
            inliers = event.get("quality_inliers")
            rows.append({
                "stamp": stamp, "stage": event.get("stage", sample[8]),
                "ground_truth_translation_delta_m": gt_delta,
                "ground_truth_yaw_delta_deg": math.degrees(gt_yaw_delta),
                "estimated_translation_delta_m": estimate_delta,
                "estimated_yaw_delta_deg": math.degrees(estimate_yaw_delta),
                "image_timestamp_delta_s": image_delta,
                "wall_clock_delta_s": wall_delta,
                "ground_truth_horizontal_speed_mps": gt["derived_horizontal_speed"],
                "ground_truth_vertical_speed_mps": gt["derived_vz"],
                "ground_truth_yaw_rate_deg_s": gt["derived_yaw_rate_deg_s"],
                "roll_deg": gt["roll_deg"], "pitch_deg": gt["pitch_deg"],
                "accel_x": gt["derived_accel_x"],
                "accel_y": gt["derived_accel_y"],
                "accel_z": gt["derived_accel_z"],
                "accel_magnitude": math.sqrt(
                    gt["derived_accel_x"] ** 2
                    + gt["derived_accel_y"] ** 2
                    + gt["derived_accel_z"] ** 2),
                "yaw_accel_deg_s2": gt["derived_yaw_accel_deg_s2"],
                "quality": inliers if inliers is not None else "",
                "features": event.get("features", ""),
                "matches": matches if matches is not None else "",
                "inliers": inliers if inliers is not None else "",
                "inlier_ratio": (
                    float(inliers) / float(matches)
                    if matches and inliers is not None else ""),
                "words": event.get("words", ""),
                "covariance_translation_trace": event.get(
                    "covariance_translation_trace", ""),
                "front_end_time_ms": event.get("front_end_time_ms", ""),
                "end_to_end_latency_ms": event.get("end_to_end_latency_ms", ""),
            })
            previous = current
        return rows

    def _correlations(self, rows):
        inputs = {
            "horizontal_speed": "ground_truth_horizontal_speed_mps",
            "vertical_speed": "ground_truth_vertical_speed_mps",
            "yaw_rate": "ground_truth_yaw_rate_deg_s",
            "roll": "roll_deg", "pitch": "pitch_deg",
            "per_frame_translation": "ground_truth_translation_delta_m",
            "per_frame_yaw": "ground_truth_yaw_delta_deg",
            "acceleration": "accel_magnitude",
            "yaw_acceleration": "yaw_accel_deg_s2",
        }
        outputs = (
            "features", "quality", "matches", "inliers", "inlier_ratio",
            "covariance_translation_trace", "front_end_time_ms",
            "end_to_end_latency_ms",
        )
        result = []
        for input_name, input_key in inputs.items():
            for output_key in outputs:
                pairs = []
                for row in rows:
                    try:
                        x = float(row[input_key])
                        y = float(row[output_key])
                    except (KeyError, TypeError, ValueError):
                        continue
                    if math.isfinite(x) and math.isfinite(y):
                        pairs.append((x, y))
                correlation = float("nan")
                if len(pairs) >= 5:
                    x = np.asarray([pair[0] for pair in pairs])
                    y = np.asarray([pair[1] for pair in pairs])
                    if np.std(x) > 1.0e-12 and np.std(y) > 1.0e-12:
                        correlation = float(np.corrcoef(x, y)[0, 1])
                result.append({
                    "input": input_name, "output": output_key,
                    "pearson_r": correlation, "samples": len(pairs),
                })
        return result

    def _finish(self):
        if self.done:
            return
        super()._finish()
        command_fields = [
            "stamp", "arrival_steady_s", "stage", "vx", "vy", "vz",
            "horizontal_speed", "vertical_speed", "yaw_rate_rad_s",
            "yaw_rate_deg_s",
        ]
        write_csv(
            self.output_dir / "commanded_motion.csv",
            command_fields, self.commanded_motion)

        motion_fields = [
            "stamp", "raw_header_stamp", "arrival_steady_s", "stage",
            "phase", "x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg",
            "reported_vx", "reported_vy", "reported_vz",
            "reported_horizontal_speed", "reported_yaw_rate_deg_s",
            "derived_vx", "derived_vy", "derived_vz",
            "derived_horizontal_speed", "derived_yaw_rate_deg_s",
            "derived_accel_x", "derived_accel_y", "derived_accel_z",
            "derived_yaw_accel_deg_s2",
            "commanded_horizontal_speed", "commanded_vertical_speed",
            "commanded_yaw_rate_deg_s",
        ]
        for stream in ("ground_truth", "mavros"):
            rows = self.motion_records[stream]
            self._smooth_motion(rows)
            self._associate_commands(rows)
            write_csv(
                self.output_dir / f"{stream}_motion.csv", motion_fields, rows)
        imu_fields = [
            "stamp", "arrival_steady_s", "stage", "roll_deg", "pitch_deg",
            "yaw_deg", "linear_accel_x", "linear_accel_y", "linear_accel_z",
            "angular_x_deg_s", "angular_y_deg_s", "angular_z_deg_s",
        ]
        write_csv(self.output_dir / "imu_motion.csv", imu_fields, self.imu_records)

        frame_rows = self._processed_frame_rows()
        frame_fields = [
            "stamp", "stage", "ground_truth_translation_delta_m",
            "ground_truth_yaw_delta_deg", "estimated_translation_delta_m",
            "estimated_yaw_delta_deg", "image_timestamp_delta_s",
            "wall_clock_delta_s", "ground_truth_horizontal_speed_mps",
            "ground_truth_vertical_speed_mps", "ground_truth_yaw_rate_deg_s",
            "roll_deg", "pitch_deg", "accel_x", "accel_y", "accel_z",
            "accel_magnitude", "yaw_accel_deg_s2",
            "quality", "features", "matches", "inliers", "inlier_ratio",
            "words", "covariance_translation_trace", "front_end_time_ms",
            "end_to_end_latency_ms",
        ]
        write_csv(
            self.output_dir / "per_processed_frame_motion.csv",
            frame_fields, frame_rows)
        write_csv(
            self.output_dir / "rtab_metrics.csv", frame_fields, frame_rows)
        write_csv(
            self.output_dir / "speed_correlations.csv",
            ["input", "output", "pearson_r", "samples"],
            self._correlations(frame_rows))

        summary_path = self.output_dir / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        speed = self._motion_summary(self.motion_records["ground_truth"])
        speed["profile"] = self.speed_test_profile
        speed["commanded_horizontal_speed_mps"] = self.target_horizontal
        speed["commanded_vertical_speed_mps"] = self.target_vertical
        speed["commanded_yaw_rate_deg_s"] = self.target_yaw_deg_s
        active_frames = [row for row in frame_rows if self._active_stage(row["stage"])]
        active_motion = [
            row for row in self.motion_records["ground_truth"]
            if self._active_stage(row["stage"])]
        speed["actual_horizontal_speed"] = finite_summary([
            abs(row["derived_horizontal_speed"]) for row in active_motion])
        speed["actual_vertical_speed"] = finite_summary([
            abs(row["derived_vz"]) for row in active_motion])
        speed["actual_yaw_rate_deg_s"] = finite_summary([
            abs(row["derived_yaw_rate_deg_s"]) for row in active_motion])
        speed["processed_frame_translation"] = finite_summary([
            row["ground_truth_translation_delta_m"] for row in active_frames])
        speed["processed_frame_yaw_delta_deg"] = finite_summary([
            row["ground_truth_yaw_delta_deg"] for row in active_frames])
        speed["inliers"] = finite_summary([
            float(row["inliers"]) for row in active_frames
            if row["inliers"] != ""])
        speed["matches"] = finite_summary([
            float(row["matches"]) for row in active_frames
            if row["matches"] != ""])
        speed["inlier_ratio"] = finite_summary([
            float(row["inlier_ratio"]) for row in active_frames
            if row["inlier_ratio"] != ""])
        vertical_rows = self.motion_records["ground_truth"]
        speed["takeoff_vertical_speed"] = finite_summary([
            abs(row["derived_vz"]) for row in vertical_rows
            if "guided_arm_takeoff" in row["stage"]])
        speed["landing_vertical_speed"] = finite_summary([
            abs(row["derived_vz"]) for row in vertical_rows
            if row["stage"].endswith(":land")])
        inherited = summary["classification"]
        classification_reasons = []
        if summary.get("fail_reasons"):
            classification = "FAIL"
            classification_reasons.extend(summary["fail_reasons"])
        elif not speed["valid_speed_experiment"]:
            classification = "NOT_EXERCISED"
            classification_reasons.append(
                "actual motion did not remain above 80% of target for 1 second")
        else:
            classification = "PASS"
            if speed.get("inliers") and speed["inliers"].get("p05", 0.0) < 20.0:
                classification = "WARN"
                classification_reasons.append("active-motion inlier p5 fell below 20")
            trajectory = summary.get("trajectory", {})
            if trajectory.get("ate_rmse_m", 0.0) > 0.10:
                classification = "WARN"
                classification_reasons.append("ATE exceeded 10 cm")
            if trajectory.get("rpe_translation_rmse_m", 0.0) > 0.03:
                classification = "WARN"
                classification_reasons.append("RPE exceeded 3 cm")
            latency = summary.get("rtab", {}).get("latency_ms") or {}
            if latency.get("p95", 0.0) > 150.0:
                classification = "WARN"
                classification_reasons.append("end-to-end latency p95 exceeded 150 ms")
        speed["classification_reasons"] = classification_reasons
        summary["robustness_classification"] = inherited
        summary["classification"] = classification
        summary["speed_envelope"] = speed
        summary_path.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        summary_md = self.output_dir / "summary.md"
        with summary_md.open("a", encoding="utf-8") as handle:
            handle.write("\n## Speed-envelope validity\n\n")
            handle.write(f"Final classification: **{classification}**\n\n")
            handle.write("| Metric | Result |\n|---|---:|\n")
            handle.write(f"| speed profile | {self.speed_test_profile} |\n")
            handle.write(f"| target | {speed['target']:.3f} |\n")
            handle.write(f"| actual mean / median | {speed.get('mean', float('nan')):.3f} / "
                         f"{speed.get('median', float('nan')):.3f} |\n")
            handle.write(f"| actual p5 / p95 / max | {speed.get('p05', float('nan')):.3f} / "
                         f"{speed.get('p95', float('nan')):.3f} / "
                         f"{speed.get('max', float('nan')):.3f} |\n")
            handle.write("| sustained >=80% target | "
                         f"{speed['sustained_above_80_percent_s']:.3f} s |\n")
            if self.speed_test_profile == "combined":
                handle.write("| simultaneous horizontal+yaw >=80% | "
                             f"{speed['simultaneous_above_80_percent_s']:.3f} s |\n")
            handle.write(f"| valid speed experiment | "
                         f"{speed['valid_speed_experiment']} |\n")


def main(args=None):
    rclpy.init(args=args)
    node = D435iSpeedEnvelopeProfiler()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.stop_requested:
            executor.spin_once(timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        executor.remove_node(node)
        if not node.done:
            node._finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
