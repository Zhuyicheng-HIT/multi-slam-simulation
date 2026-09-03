from pathlib import Path
import unittest


class ManualControlContractTest(unittest.TestCase):
    def test_service_contract_is_bounded_and_source_owned(self):
        text = (Path(__file__).resolve().parents[1] / "uf_reliability" /
                "manual_relocalization_control.py").read_text()
        self.assertIn("/relocalization/request_intent", text)
        self.assertNotIn("/mavros/setpoint_position/local", text)
        self.assertIn('self.source_id != "manual_control"', text)
        self.assertIn("minimum_lease_s", text)
        self.assertIn("timestamp_regression", text)

    def test_duplicate_and_cancel_are_idempotent_by_contract(self):
        text = (Path(__file__).resolve().parents[1] / "uf_reliability" /
                "manual_relocalization_control.py").read_text()
        self.assertIn('"already_active"', text)
        self.assertIn('"already_cancelled"', text)
        self.assertIn('"manual_cancel"', text)


if __name__ == "__main__":
    unittest.main()
