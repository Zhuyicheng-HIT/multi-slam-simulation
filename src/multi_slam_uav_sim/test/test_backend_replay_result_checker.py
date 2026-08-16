import importlib.util
from pathlib import Path
import unittest


TOOL_PATH = Path(__file__).resolve().parents[3] / "tools" / "check_backend_replay_result.py"
SPEC = importlib.util.spec_from_file_location("check_backend_replay_result", TOOL_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _summary(**overrides):
    values = {
        "native_received": "100",
        "optimized_states_committed": "97",
        "imu_pair_timeouts": "0",
        "native_queue_overflow": "0",
        "native_queue_discarded": "0",
        "native_consumed_without_state_commit": "3",
        "native_worker_errors": "0",
        "optimization_errors": "0",
        "scan_cache_hits": "0",
    }
    values.update({key: str(value) for key, value in overrides.items()})
    return values


class BackendReplayResultCheckerTest(unittest.TestCase):
    def test_accepts_native_only_replay_at_reference_commit_count(self):
        report = MODULE.evaluate_lossless_replay(
            _summary(),
            expected_native_count=100,
            expected_scan_request_count=0,
            expected_committed_count=97,
        )
        self.assertTrue(report["passed"])
        self.assertFalse(report["prediction_chain_required"])

    def test_requires_recorded_prediction_chain_when_present(self):
        report = MODULE.evaluate_lossless_replay(
            _summary(scan_cache_hits=2),
            expected_native_count=100,
            expected_scan_request_count=10,
            expected_committed_count=97,
        )
        self.assertFalse(report["passed"])
        self.assertFalse(
            report["gates"]["prediction_chain_complete_or_not_recorded"]
        )

    def test_rejects_commit_regression_and_imu_timeout(self):
        report = MODULE.evaluate_lossless_replay(
            _summary(
                optimized_states_committed=96,
                native_consumed_without_state_commit=4,
                imu_pair_timeouts=1,
            ),
            expected_native_count=100,
            expected_committed_count=97,
        )
        self.assertFalse(report["passed"])
        self.assertFalse(
            report["gates"]["committed_at_least_reference_trajectory"]
        )
        self.assertFalse(report["gates"]["zero_imu_pair_timeouts"])

    def test_auxiliary_mode_accepts_states_beyond_native_packet_count(self):
        report = MODULE.evaluate_lossless_replay(
            _summary(
                optimized_states_committed=125,
                native_consumed_without_state_commit=3,
                auxiliary_keyframe_committed=28,
                auxiliary_keyframe_rejected=0,
                auxiliary_keyframe_errors=0,
            ),
            expected_native_count=100,
            expected_committed_count=125,
            allow_auxiliary_keyframes=True,
            maximum_uncommitted_native_count=3,
        )
        self.assertTrue(report["passed"])
        self.assertTrue(report["auxiliary_keyframes_allowed"])

    def test_auxiliary_mode_rejects_missing_or_failed_auxiliary_states(self):
        report = MODULE.evaluate_lossless_replay(
            _summary(
                optimized_states_committed=100,
                native_consumed_without_state_commit=3,
                auxiliary_keyframe_committed=0,
                auxiliary_keyframe_rejected=1,
                auxiliary_keyframe_errors=1,
            ),
            expected_native_count=100,
            expected_committed_count=100,
            allow_auxiliary_keyframes=True,
            maximum_uncommitted_native_count=3,
        )
        self.assertFalse(report["passed"])
        self.assertFalse(
            report["gates"]["auxiliary_keyframes_were_committed"]
        )
        self.assertFalse(report["gates"]["zero_auxiliary_keyframe_errors"])


if __name__ == "__main__":
    unittest.main()
