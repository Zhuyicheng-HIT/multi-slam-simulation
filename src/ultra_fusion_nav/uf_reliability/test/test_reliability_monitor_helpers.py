import unittest
from types import SimpleNamespace

from uf_reliability.reliability_monitor import (
    flow_observation_valid,
    nonnegative_diagnostic_value,
)


class ReliabilityMonitorHelpersTest(unittest.TestCase):
    @staticmethod
    def message(value, status_name="unified_backend_fusion"):
        return SimpleNamespace(status=[SimpleNamespace(
            name=status_name,
            values=[SimpleNamespace(
                key="imu_preintegration_residual_mahalanobis",
                value=value,
            )],
        )])

    def test_reads_backend_preintegration_residual(self):
        value = nonnegative_diagnostic_value(
            self.message("1.25"),
            "unified_backend_fusion",
            "imu_preintegration_residual_mahalanobis",
        )
        self.assertEqual(value, 1.25)

    def test_rejects_missing_stale_sentinel_and_nonfinite_values(self):
        for value in ("-1", "nan", "not-a-number"):
            result = nonnegative_diagnostic_value(
                self.message(value),
                "unified_backend_fusion",
                "imu_preintegration_residual_mahalanobis",
            )
            self.assertIsNone(result)
        self.assertIsNone(nonnegative_diagnostic_value(
            self.message("1.0", status_name="other_node"),
            "unified_backend_fusion",
            "imu_preintegration_residual_mahalanobis",
        ))

    def test_rotation_hard_gate_marks_flow_unavailable(self):
        enabled_gate = SimpleNamespace(hard_disabled=False)
        disabled_gate = SimpleNamespace(hard_disabled=True)
        self.assertTrue(flow_observation_valid(True, 0.05, enabled_gate))
        self.assertFalse(flow_observation_valid(True, 0.35, disabled_gate))
        self.assertFalse(flow_observation_valid(True, None, enabled_gate))
        self.assertFalse(flow_observation_valid(False, 0.05, enabled_gate))


if __name__ == "__main__":
    unittest.main()
