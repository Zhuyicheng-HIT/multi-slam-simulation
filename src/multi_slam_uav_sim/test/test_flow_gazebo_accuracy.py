from types import SimpleNamespace
import unittest

from multi_slam_uav_sim.flow_gazebo_accuracy import (
    accuracy_row_arrays,
    select_association_basis,
    stamp_seconds,
)


class FlowGazeboAccuracyTest(unittest.TestCase):
    def test_stamp_seconds_accepts_ros_header_nanosec(self):
        stamp = SimpleNamespace(sec=12, nanosec=345000000)
        self.assertEqual(stamp_seconds(stamp), 12.345)

    def test_stamp_seconds_accepts_gazebo_nsec(self):
        stamp = SimpleNamespace(sec=7, nsec=250000000)
        self.assertEqual(stamp_seconds(stamp), 7.25)

    def test_stamp_seconds_rejects_missing_or_zero_stamp(self):
        self.assertEqual(stamp_seconds(SimpleNamespace(sec=0, nanosec=0)), 0.0)
        self.assertEqual(stamp_seconds(SimpleNamespace()), 0.0)

    def test_accuracy_row_arrays_preserves_truth_estimate_and_quality_columns(self):
        rows = [(10.0, 100.0, 0.032, 0.034, 1.0, -2.0, 0.8, -1.6, 187.0, 2.6)]
        estimates, truth, integration, arrival_intervals, quality, distance = accuracy_row_arrays(rows)
        self.assertEqual(estimates.tolist(), [[0.8, -1.6]])
        self.assertEqual(truth.tolist(), [[1.0, -2.0]])
        self.assertEqual(integration.tolist(), [0.032])
        self.assertEqual(arrival_intervals.tolist(), [0.034])
        self.assertEqual(quality.tolist(), [187.0])
        self.assertEqual(distance.tolist(), [2.6])

    def test_association_uses_source_stamp_for_shared_clock(self):
        self.assertEqual(
            select_association_basis([10.0, 10.1, 10.2], [10.02, 10.12]),
            "source_stamp",
        )

    def test_association_uses_arrival_for_wall_and_sim_clocks(self):
        self.assertEqual(
            select_association_basis(
                [10.0, 10.1, 10.2], [1_785_160_000.0, 1_785_160_000.1]
            ),
            "arrival",
        )


if __name__ == "__main__":
    unittest.main()
