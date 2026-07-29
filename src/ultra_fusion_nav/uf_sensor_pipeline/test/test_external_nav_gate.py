import unittest

from uf_sensor_pipeline.external_nav_gate import scheduler_state_allowed


class SchedulerStateGateTest(unittest.TestCase):
    def test_only_explicit_control_safe_states_are_allowed(self):
        allowed = ("NORMAL", "RECOVERED")
        self.assertTrue(scheduler_state_allowed("NORMAL", allowed))
        self.assertTrue(scheduler_state_allowed("recovered", allowed))
        self.assertFalse(scheduler_state_allowed("DEGRADED", allowed))
        self.assertFalse(scheduler_state_allowed("FAILSAFE", allowed))
        self.assertFalse(scheduler_state_allowed("", allowed))


if __name__ == "__main__":
    unittest.main()
