import importlib.util
from pathlib import Path
import unittest


def load_gate():
    script = Path(__file__).resolve().parents[1] / "scripts" / "evaluate_optical_flow_gate.py"
    spec = importlib.util.spec_from_file_location("evaluate_optical_flow_gate", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GATE = load_gate()


class OpticalFlowGateTest(unittest.TestCase):
    def _report(self, valid):
        return {"trajectory": {"coupling_reference_valid": valid}}

    def test_both_independent_checks_pass(self):
        result = GATE.evaluate_gate(
            {"passed": True}, self._report(True), "FLOW_ACCURACY passed=True"
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["classification"], "sensor_and_lio_crosscheck_passed")

    def test_unhealthy_lio_reference_is_inconclusive_not_sensor_failure(self):
        result = GATE.evaluate_gate(
            {"passed": False}, self._report(False), "FLOW_ACCURACY passed=True"
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["classification"], "sensor_passed_lio_crosscheck_inconclusive")

    def test_healthy_lio_reference_can_reject_sensor_crosscheck(self):
        result = GATE.evaluate_gate(
            {"passed": False}, self._report(True), "FLOW_ACCURACY passed=True"
        )
        self.assertFalse(result["passed"])

    def test_gazebo_sensor_failure_is_always_fatal(self):
        result = GATE.evaluate_gate(
            {"passed": True}, self._report(True), "FLOW_ACCURACY passed=False"
        )
        self.assertFalse(result["passed"])


if __name__ == "__main__":
    unittest.main()
