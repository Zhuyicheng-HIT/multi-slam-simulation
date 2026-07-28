import json
import math
import os
import threading
import time
from collections import deque

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node as GzNode
from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix, PointCloud2


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]


class TopicWindow:
    def __init__(self, size=4000):
        self.samples = deque(maxlen=size)
        self.total = 0

    def add(self, arrival_s, source_stamp_s=None):
        self.samples.append((float(arrival_s), source_stamp_s))
        self.total += 1

    def summary(self, now_s, window_s):
        recent = [sample for sample in self.samples if now_s - sample[0] <= window_s]
        arrivals = [sample[0] for sample in recent]
        intervals = [
            after - before for before, after in zip(arrivals, arrivals[1:])
        ]
        rate_hz = 0.0
        if len(arrivals) >= 2:
            rate_hz = (len(arrivals) - 1) / max(1.0e-6, arrivals[-1] - arrivals[0])
        source_stamps = [
            float(sample[1]) for sample in recent
            if sample[1] is not None and math.isfinite(float(sample[1]))
        ]
        source_rate_hz = 0.0
        if len(source_stamps) >= 2 and source_stamps[-1] > source_stamps[0]:
            source_rate_hz = (len(source_stamps) - 1) / (source_stamps[-1] - source_stamps[0])
        mean_interval = sum(intervals) / len(intervals) if intervals else 0.0
        median_interval = percentile(intervals, 0.50)
        jitter_ms = 0.0
        if intervals:
            jitter_ms = math.sqrt(
                sum((value - mean_interval) ** 2 for value in intervals) / len(intervals)
            ) * 1000.0
        return {
            "total_messages": self.total,
            "window_messages": len(recent),
            "rate_hz": rate_hz,
            "source_stamp_rate_hz": source_rate_hz,
            "source_to_arrival_rate_ratio": (
                source_rate_hz / rate_hz if source_rate_hz > 0.0 and rate_hz > 0.0 else None
            ),
            "arrival_interval_median_ms": median_interval * 1000.0,
            "jitter_ms": jitter_ms,
            "age_s": None if not arrivals else now_s - arrivals[-1],
        }


BACKEND_TIMING_KEYS = frozenset({
    "backend_solve_ms",
    "backend_solve_mean_ms",
    "backend_solve_max_ms",
    "callback_ms",
})


def diagnostic_timing_values(message):
    values = {}
    for status in message.status:
        for item in status.values:
            if not (
                item.key.startswith("timing_")
                or item.key in BACKEND_TIMING_KEYS
            ):
                continue
            try:
                values[f"{status.name}/{item.key}"] = float(item.value)
            except ValueError:
                continue
    return values


class SimulationPerformanceMonitor(Node):
    """Measure simulation RTF, topic rates, and approximate pipeline-stage latency."""

    def __init__(self):
        super().__init__("simulation_performance_monitor")
        self.declare_parameter("world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("window_s", 10.0)
        self.declare_parameter("report_period_s", 5.0)
        self.declare_parameter("output_path", "")
        self.declare_parameter("minimum_live_rtf", 0.80)
        self.declare_parameter("minimum_flow_rate_hz", 10.0)
        self.declare_parameter("minimum_gnss_rate_hz", 4.0)
        self.declare_parameter("minimum_external_nav_rate_hz", 10.0)
        self.declare_parameter("flow_truth_assistance", False)
        self.declare_parameter("fusion_topic", "/fusion/gps_flow/odom")
        self.declare_parameter(
            "fusion_diagnostic_topic", "/fusion/gps_flow/diagnostics"
        )
        self.world_name = str(self.get_parameter("world_name").value)
        self.window_s = float(self.get_parameter("window_s").value)
        self.output_path = str(self.get_parameter("output_path").value)
        self.minimum_live_rtf = float(self.get_parameter("minimum_live_rtf").value)
        self.minimum_rates = {
            "raw_flow": float(self.get_parameter("minimum_flow_rate_hz").value),
            "gnss": float(self.get_parameter("minimum_gnss_rate_hz").value),
            "external_nav": float(self.get_parameter("minimum_external_nav_rate_hz").value),
        }
        self.flow_truth_assistance = bool(
            self.get_parameter("flow_truth_assistance").value)
        self.fusion_topic = str(self.get_parameter("fusion_topic").value)
        self.fusion_diagnostic_topic = str(
            self.get_parameter("fusion_diagnostic_topic").value
        )
        self.started_s = time.monotonic()
        self.lock = threading.Lock()
        self.topics = {
            name: TopicWindow()
            for name in (
                "flow_image", "raw_flow", "sensor_flow", "gnss", "fusion",
                "external_nav", "lidar", "d435_color", "d435_depth",
            )
        }
        self.latest_arrival = {}
        self.stage_latency_ms = {
            name: deque(maxlen=2000)
            for name in (
                "flow_image_to_raw_flow", "raw_to_sensor_flow",
                "sensor_flow_to_fusion", "gnss_to_fusion", "fusion_to_external_nav",
            )
        }
        self.rtf_samples = deque(maxlen=2000)
        self.sim_step_samples_ms = deque(maxlen=2000)
        self.flow_integration_ms = deque(maxlen=2000)
        self.node_timings_ms = {}
        self.last_report = {}

        self.create_subscription(
            Image, "/camera/camera/color/image_raw",
            lambda msg: self._record("flow_image", msg), qos_profile_sensor_data)
        self.create_subscription(
            OpticalFlowRad, "/sim/optical_flow/rad",
            self._raw_flow,
            qos_profile_sensor_data)
        self.create_subscription(
            OpticalFlowRad, "/sensors/optical_flow/rad",
            lambda msg: self._record(
                "sensor_flow", msg, "raw_flow", "raw_to_sensor_flow"),
            qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, "/sensors/gnss/fix",
            lambda msg: self._record("gnss", msg), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.fusion_topic, self._fusion, 20)
        self.create_subscription(
            Odometry, "/mavros/odometry/out",
            lambda msg: self._record(
                "external_nav", msg, "fusion", "fusion_to_external_nav"), 20)
        self.create_subscription(
            PointCloud2, "/sim/mid360/points_raw",
            lambda msg: self._record("lidar", msg), qos_profile_sensor_data)
        self.create_subscription(
            Image, "/front/d435i/color/image_raw",
            lambda msg: self._record("d435_color", msg), qos_profile_sensor_data)
        self.create_subscription(
            Image, "/front/d435i/depth/image_rect_raw",
            lambda msg: self._record("d435_depth", msg), qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, self.fusion_diagnostic_topic,
            self._node_diagnostics, 10)
        self.create_subscription(
            DiagnosticArray, "/external_nav/diagnostics", self._node_diagnostics, 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/simulation/performance", 10)
        self.gz_node = GzNode()
        self.gz_node.subscribe(
            WorldStatistics, f"/world/{self.world_name}/stats", self._world_stats)
        self.create_timer(
            max(1.0, float(self.get_parameter("report_period_s").value)),
            self._publish_report)
        self.get_logger().info(
            f"Simulation performance monitor active for world={self.world_name}; "
            f"fusion={self.fusion_topic}; "
            f"report={self.output_path or 'diagnostics_only'}")

    @staticmethod
    def _source_stamp(message):
        try:
            stamp = message.header.stamp
            value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
            return value if value > 0.0 else None
        except Exception:
            return None

    def _record(self, name, message=None, upstream=None, stage=None):
        now = time.monotonic()
        with self.lock:
            self.topics[name].add(now, self._source_stamp(message))
            self.latest_arrival[name] = now
            if upstream is not None and stage is not None:
                upstream_time = self.latest_arrival.get(upstream)
                if upstream_time is not None:
                    latency_s = now - upstream_time
                    if 0.0 <= latency_s <= 1.0:
                        self.stage_latency_ms[stage].append(latency_s * 1000.0)

    def _fusion(self, msg):
        self._record("fusion", msg, "sensor_flow", "sensor_flow_to_fusion")
        now = time.monotonic()
        with self.lock:
            gnss_time = self.latest_arrival.get("gnss")
            if gnss_time is not None and 0.0 <= now - gnss_time <= 1.0:
                self.stage_latency_ms["gnss_to_fusion"].append((now - gnss_time) * 1000.0)

    def _raw_flow(self, msg):
        self._record("raw_flow", msg, "flow_image", "flow_image_to_raw_flow")
        integration_ms = float(msg.integration_time_us) * 1.0e-3
        if math.isfinite(integration_ms) and integration_ms > 0.0:
            with self.lock:
                self.flow_integration_ms.append(integration_ms)

    def _world_stats(self, msg):
        with self.lock:
            rtf = float(msg.real_time_factor)
            if math.isfinite(rtf) and rtf >= 0.0:
                self.rtf_samples.append(rtf)
            step_ms = (
                float(msg.step_size.sec) * 1000.0
                + float(msg.step_size.nsec) * 1.0e-6
            )
            if math.isfinite(step_ms) and step_ms >= 0.0:
                self.sim_step_samples_ms.append(step_ms)

    def _node_diagnostics(self, msg):
        with self.lock:
            self.node_timings_ms.update(diagnostic_timing_values(msg))

    def _build_report(self):
        now = time.monotonic()
        with self.lock:
            topic_report = {
                name: window.summary(now, self.window_s)
                for name, window in self.topics.items()
            }
            stage_report = {
                name: {
                    "samples": len(values),
                    "p50_ms": percentile(values, 0.50),
                    "p95_ms": percentile(values, 0.95),
                    "max_ms": max(values) if values else 0.0,
                    "approximate_from_arrival_times": True,
                }
                for name, values in self.stage_latency_ms.items()
            }
            rtf = list(self.rtf_samples)
            step = list(self.sim_step_samples_ms)
            flow_integration = list(self.flow_integration_ms)
            node_timings = dict(self.node_timings_ms)
        wait_bottleneck = max(
            stage_report,
            key=lambda name: stage_report[name]["p95_ms"],
            default="insufficient_samples",
        )
        mean_timings = {
            name: value for name, value in node_timings.items()
            if name.endswith("_mean_ms")
        }
        compute_bottleneck = max(
            mean_timings, key=mean_timings.get, default="insufficient_samples")
        rtf_median = percentile(rtf, 0.50)
        rates_ok = all(
            topic_report[name]["rate_hz"] >= minimum
            for name, minimum in self.minimum_rates.items()
        )
        flow_ratio = topic_report["raw_flow"]["source_to_arrival_rate_ratio"]
        flow_source_rate_valid = (
            flow_ratio is not None and 0.75 <= flow_ratio <= 1.25
        )
        flow_integration_median_ms = percentile(flow_integration, 0.50)
        flow_arrival_interval_ms = topic_report["raw_flow"][
            "arrival_interval_median_ms"]
        flow_integration_arrival_ratio = (
            flow_integration_median_ms / flow_arrival_interval_ms
            if flow_integration_median_ms > 0.0 and flow_arrival_interval_ms > 0.0
            else None
        )
        flow_integration_valid = (
            flow_integration_arrival_ratio is not None
            and 0.75 <= flow_integration_arrival_ratio <= 1.25
        )
        live_timing_valid = (
            rtf_median >= self.minimum_live_rtf
            and rates_ok
            and flow_source_rate_valid
            and flow_integration_valid
        )
        algorithm_accuracy_valid = live_timing_valid and not self.flow_truth_assistance
        return {
            "schema_version": 1,
            "fusion_topic": self.fusion_topic,
            "fusion_diagnostic_topic": self.fusion_diagnostic_topic,
            "wall_duration_s": now - self.started_s,
            "simulation": {
                "world": self.world_name,
                "real_time_factor_median": rtf_median,
                "real_time_factor_p10": percentile(rtf, 0.10),
                "real_time_factor_min": min(rtf) if rtf else 0.0,
                "step_size_median_ms": percentile(step, 0.50),
                "samples": len(rtf),
            },
            "topics": topic_report,
            "stages": stage_report,
            "node_timings_ms": node_timings,
            "optical_flow_timing": {
                "integration_time_median_ms": flow_integration_median_ms,
                "arrival_interval_median_ms": flow_arrival_interval_ms,
                "integration_to_arrival_ratio": flow_integration_arrival_ratio,
            },
            "bottleneck_wait_stage_by_p95": wait_bottleneck,
            "bottleneck_compute_stage_by_mean": compute_bottleneck,
            "gates": {
                "live_timing_comparison_valid": live_timing_valid,
                "algorithm_accuracy_comparison_valid": algorithm_accuracy_valid,
                "flow_truth_assistance_enabled": self.flow_truth_assistance,
                "minimum_live_rtf": self.minimum_live_rtf,
                "minimum_rates_hz": self.minimum_rates,
                "flow_source_arrival_ratio_valid": flow_source_rate_valid,
                "flow_integration_arrival_ratio_valid": flow_integration_valid,
                "algorithm_accuracy_still_requires_truth_ATE_RPE": True,
                "offline_rosbag_comparison_allowed_when_live_gate_fails": True,
            },
        }

    def _write_report(self, report):
        if not self.output_path:
            return
        directory = os.path.dirname(os.path.abspath(self.output_path))
        os.makedirs(directory, exist_ok=True)
        temporary = self.output_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, self.output_path)

    @staticmethod
    def _value(key, value):
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _publish_report(self):
        report = self._build_report()
        self.last_report = report
        self._write_report(report)
        valid = report["gates"]["algorithm_accuracy_comparison_valid"]
        status = DiagnosticStatus()
        status.name = "simulation/performance_gate"
        status.hardware_id = self.world_name
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        status.message = (
            "algorithm_comparison_valid" if valid
            else "timing_or_truth_assistance_gate_failed")
        status.values = [
            self._value(
                "real_time_factor_median",
                f"{report['simulation']['real_time_factor_median']:.3f}"),
            self._value(
                "bottleneck_wait_stage", report["bottleneck_wait_stage_by_p95"]),
            self._value(
                "bottleneck_compute_stage",
                report["bottleneck_compute_stage_by_mean"]),
            self._value(
                "flow_truth_assistance_enabled",
                report["gates"]["flow_truth_assistance_enabled"]),
            self._value(
                "raw_flow_rate_hz", f"{report['topics']['raw_flow']['rate_hz']:.3f}"),
            self._value(
                "raw_flow_source_rate_hz",
                f"{report['topics']['raw_flow']['source_stamp_rate_hz']:.3f}"),
            self._value(
                "raw_flow_source_arrival_ratio",
                report["topics"]["raw_flow"]["source_to_arrival_rate_ratio"]),
            self._value(
                "flow_integration_arrival_ratio",
                report["optical_flow_timing"]["integration_to_arrival_ratio"]),
            self._value("gnss_rate_hz", f"{report['topics']['gnss']['rate_hz']:.3f}"),
            self._value(
                "external_nav_rate_hz",
                f"{report['topics']['external_nav']['rate_hz']:.3f}"),
        ]
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status.append(status)
        self.diagnostic_pub.publish(output)
        self.get_logger().info(
            "PERFORMANCE "
            f"rtf={report['simulation']['real_time_factor_median']:.3f} "
            f"flow={report['topics']['raw_flow']['rate_hz']:.2f}Hz "
            f"gnss={report['topics']['gnss']['rate_hz']:.2f}Hz "
            f"external_nav={report['topics']['external_nav']['rate_hz']:.2f}Hz "
            f"wait_bottleneck={report['bottleneck_wait_stage_by_p95']} "
            f"compute_bottleneck={report['bottleneck_compute_stage_by_mean']} "
            f"live_valid={valid}")

    def final_report(self):
        report = self._build_report()
        self._write_report(report)
        return report


def main(args=None):
    rclpy.init(args=args)
    node = SimulationPerformanceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.final_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
