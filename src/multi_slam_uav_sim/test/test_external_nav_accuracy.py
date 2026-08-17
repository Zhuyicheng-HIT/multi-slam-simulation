import csv
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np
from rclpy.qos import ReliabilityPolicy

from multi_slam_uav_sim.external_nav_accuracy import ExternalNavAccuracy
from multi_slam_uav_sim.simulation_performance_monitor import (
    TopicWindow,
    diagnostic_timing_values,
    read_cpu_frequency_khz,
    read_system_memory_usage,
    system_cpu_utilization_percent,
    topic_rate_for_gate,
)


class ExternalNavAccuracyTest(unittest.TestCase):
    def test_truth_subscription_qos_defaults_to_best_effort_compatibility(
            self):
        qos = ExternalNavAccuracy._truth_subscription_qos("best_effort", 7)

        self.assertEqual(qos.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(qos.depth, 7)
        with self.assertRaises(ValueError):
            ExternalNavAccuracy._truth_subscription_qos("sometimes", 7)

    def test_cpu_frequency_reader_uses_available_cpu_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for index, value in enumerate((1800000, 2400000, 3000000)):
                path = root / f"cpu{index}" / "cpufreq"
                path.mkdir(parents=True)
                (path / "scaling_cur_freq").write_text(
                    str(value), encoding="ascii"
                )
            self.assertEqual(read_cpu_frequency_khz(root), 2400000)

    def test_system_cpu_utilization_uses_total_capacity(self):
        self.assertAlmostEqual(
            system_cpu_utilization_percent((1000, 400), (1200, 450)),
            75.0,
        )
        self.assertIsNone(system_cpu_utilization_percent(None, (1200, 450)))
        self.assertIsNone(
            system_cpu_utilization_percent((1200, 450), (1100, 460))
        )

    def test_system_memory_uses_memavailable_without_cache_guessing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "meminfo"
            path.write_text(
                "MemTotal:       8192000 kB\n"
                "MemFree:        1000000 kB\n"
                "MemAvailable:   6144000 kB\n",
                encoding="ascii",
            )
            total, used, percent = read_system_memory_usage(path)
        self.assertEqual(total, 8192000 * 1024)
        self.assertEqual(used, 2048000 * 1024)
        self.assertAlmostEqual(percent, 25.0)

    def test_performance_monitor_accepts_backend_timing_diagnostics(self):
        message = SimpleNamespace(status=[SimpleNamespace(
            name="unified_backend_fusion",
            values=[
                SimpleNamespace(key="backend_solve_mean_ms", value="16.5"),
                SimpleNamespace(key="backend_solve_max_ms", value="25.0"),
                SimpleNamespace(key="callback_ms", value="18.2"),
                SimpleNamespace(key="window_states", value="8"),
                SimpleNamespace(key="backend_solve_ms", value="not-a-number"),
            ],
        )])

        values = diagnostic_timing_values(message)

        self.assertEqual(values, {
            "unified_backend_fusion/backend_solve_mean_ms": 16.5,
            "unified_backend_fusion/backend_solve_max_ms": 25.0,
            "unified_backend_fusion/callback_ms": 18.2,
        })

    def test_topic_window_exposes_source_to_arrival_rate_mismatch(self):
        window = TopicWindow()
        for index in range(10):
            window.add(arrival_s=index * 0.1, source_stamp_s=index * 0.05)

        summary = window.summary(now_s=0.9, window_s=2.0)

        self.assertAlmostEqual(summary["rate_hz"], 10.0)
        self.assertAlmostEqual(summary["source_stamp_rate_hz"], 20.0)
        self.assertAlmostEqual(summary["source_to_arrival_rate_ratio"], 2.0)
        self.assertAlmostEqual(summary["source_interval_median_ms"], 50.0)
        self.assertAlmostEqual(summary["arrival_interval_median_ms"], 100.0)

    def test_gnss_rate_gate_uses_sim_source_time_under_low_rtf(self):
        window = TopicWindow()
        for index in range(10):
            window.add(arrival_s=index * 1.0, source_stamp_s=index * 0.4)

        summary = window.summary(now_s=9.0, window_s=10.0)

        self.assertAlmostEqual(summary["rate_hz"], 1.0)
        self.assertAlmostEqual(summary["source_stamp_rate_hz"], 2.5)
        self.assertAlmostEqual(topic_rate_for_gate("gnss", summary), 2.5)
        self.assertAlmostEqual(topic_rate_for_gate("external_nav", summary), 2.5)

    def test_rigid_alignment_recovers_rotation_without_hiding_scale(self):
        estimate = np.asarray([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
        angle = math.radians(30.0)
        rotation = np.asarray([
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)],
        ])
        truth = (rotation @ (1.2 * estimate).T).T + np.asarray([3.0, -2.0])

        aligned, recovered_rotation, _, scale = ExternalNavAccuracy._align_xy(
            estimate, truth)

        self.assertAlmostEqual(
            math.degrees(math.atan2(
                recovered_rotation[1, 0], recovered_rotation[0, 0])),
            30.0,
            places=6,
        )
        self.assertAlmostEqual(scale, 1.2, places=6)
        self.assertGreater(
            float(np.max(np.linalg.norm(aligned - truth, axis=1))), 0.1)

    def test_initial_pose_alignment_does_not_use_future_drift(self):
        times = np.arange(10, dtype=float)
        truth = np.column_stack((times, np.zeros(10), np.zeros(10)))
        estimate = truth + np.asarray([1.0, -2.0, 0.5])
        estimate[:, 0] += 0.1 * np.maximum(times - 2.0, 0.0)
        yaw = np.zeros(10)

        aligned, _, translation, z_offset, _, samples = (
            ExternalNavAccuracy._initial_pose_alignment(
                estimate, truth, yaw, yaw, times, duration_s=2.0)
        )
        summary = ExternalNavAccuracy._position_error_summary(aligned, truth)

        self.assertEqual(samples, 3)
        np.testing.assert_allclose(translation, [-1.0, 2.0])
        self.assertAlmostEqual(z_offset, -0.5)
        self.assertAlmostEqual(summary["endpoint_error_m"]["x"], 0.7)
        self.assertGreater(summary["three_dimensional"]["rmse_m"], 0.2)
        self.assertAlmostEqual(summary["vertical"]["rmse_m"], 0.0)

    def test_threshold_exceedance_tracks_contiguous_duration(self):
        summary = ExternalNavAccuracy._threshold_exceedance_summary(
            matched_times=[0.0, 0.1, 0.2, 0.3, 0.4, 0.9, 1.0, 1.1],
            errors=[0.1, 0.21, 0.22, 0.23, 0.1, 0.21, 0.22, 0.1],
            threshold_m=0.2,
            maximum_gap_s=0.2,
        )

        self.assertEqual(summary["sample_count"], 5)
        self.assertAlmostEqual(summary["sample_ratio"], 5.0 / 8.0)
        self.assertAlmostEqual(summary["maximum_contiguous_duration_s"], 0.2)
        self.assertAlmostEqual(summary["total_duration_s"], 0.3)
        self.assertAlmostEqual(summary["first_stamp_s"], 0.1)
        self.assertAlmostEqual(summary["last_stamp_s"], 1.0)

    def test_position_rpe_uses_frozen_initial_rotation(self):
        estimate = np.asarray([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
        ])
        truth = np.asarray([
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 2.0, 0.0],
        ])
        rotation = np.asarray([[0.0, -1.0], [1.0, 0.0]])

        causal = ExternalNavAccuracy._position_rpe_errors(
            estimate, truth, [0.0, 1.0, 2.0], rotation, 1.0)
        unaligned = ExternalNavAccuracy._position_rpe_errors(
            estimate, truth, [0.0, 1.0, 2.0], np.eye(2), 1.0)

        np.testing.assert_allclose(causal, [0.0, 0.0], atol=1.0e-12)
        np.testing.assert_allclose(unaligned, [math.sqrt(2.0)] * 2)

    def test_causal_samples_are_written_with_signed_residuals(self):
        rows = ExternalNavAccuracy._causal_sample_rows(
            matched_times=[1.0, 2.0],
            estimate=np.asarray([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]),
            causal_aligned=np.asarray([[0.1, -0.2, 0.3], [1.2, 0.8, 1.4]]),
            truth=np.asarray([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]),
            causal_yaw_errors=np.asarray([0.0, math.radians(-2.0)]),
            threshold_m=0.2,
        )
        self.assertAlmostEqual(rows[0]["error_x_m"], 0.1)
        self.assertAlmostEqual(rows[0]["error_y_m"], -0.2)
        self.assertAlmostEqual(rows[1]["error_z_m"], 0.4)
        self.assertAlmostEqual(rows[1]["yaw_error_deg"], -2.0)
        self.assertEqual(rows[0]["above_threshold"], 1)

        with tempfile.TemporaryDirectory() as directory:
            node = ExternalNavAccuracy.__new__(ExternalNavAccuracy)
            node.output_path = str(Path(directory) / "report.json")
            node.samples_output_path = str(Path(directory) / "samples.csv")
            node.last_causal_samples = rows
            node._write({"schema_version": 4})
            with open(
                    node.samples_output_path,
                    newline="",
                    encoding="utf-8") as stream:
                written = list(csv.DictReader(stream))

        self.assertEqual(len(written), 2)
        self.assertEqual(written[0]["stamp_s"], "1.0")
        self.assertEqual(written[0]["error_y_m"], "-0.2")
        self.assertEqual(written[1]["above_threshold"], "1")

    def test_report_uses_causal_metrics_and_keeps_legacy_metrics(self):
        node = ExternalNavAccuracy.__new__(ExternalNavAccuracy)
        node.lock = threading.Lock()
        node.maximum_pose_gap = 2.0
        node.rpe_interval = 1.0
        node.minimum_motion_speed = 0.03
        node.initial_alignment_duration = 3.0
        node.acceptance_threshold = 0.2
        node.maximum_sustained_exceedance = 0.5
        node.started_wall_s = time.monotonic()
        node.samples_output_path = ""
        node.truth_samples = [
            (float(index), float(index), 0.0, 0.0, 0.0)
            for index in range(32)
        ]
        node.odom_samples = []
        for index in range(30):
            stamp_s = float(index) + 0.5
            z_drift = 0.3 if stamp_s >= 6.5 else 0.0
            yaw = -math.pi / 2.0
            if stamp_s >= 6.5:
                yaw += math.radians(10.0)
            node.odom_samples.append(
                (stamp_s, 1.0, -stamp_s - 2.0, z_drift, yaw))

        report = node._calculate()

        self.assertEqual(report["schema_version"], 4)
        self.assertEqual(report["acceptance"]["metric_basis"],
                         "frozen_initial_alignment")
        self.assertFalse(report["acceptance"]["passed"])
        self.assertGreater(
            report["causal_ate"]["threshold_exceedance"]
            ["maximum_contiguous_duration_s"],
            20.0,
        )
        self.assertAlmostEqual(
            report["causal_ate"]["endpoint_error_m"]["z"], 0.3)
        self.assertGreater(report["yaw"]["rmse_deg"], 5.0)
        self.assertIn("legacy_aligned_yaw", report)
        self.assertIn("legacy_aligned_rpe_1s", report)
        self.assertEqual(len(node.last_causal_samples), 30)

    def test_yaw_wrap_and_quaternion_conversion_cross_branch_cut(self):
        yaw = math.radians(179.0)
        recovered = ExternalNavAccuracy._quaternion_yaw(
            0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        self.assertAlmostEqual(recovered, yaw)
        error = ExternalNavAccuracy._wrap_angle(
            math.radians(-179.0) - math.radians(179.0))
        self.assertAlmostEqual(math.degrees(error), 2.0, places=6)


if __name__ == "__main__":
    unittest.main()
