import importlib.util
import math
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[3] / "tools" / "score_relocalization_experiments.py"
SPEC = importlib.util.spec_from_file_location("relocalization_scoring", SCRIPT)
SCORING = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCORING)


class RelocalizationExperimentScoringTest(unittest.TestCase):
    def test_early_landing_is_complete_runtime_evidence(self):
        self.assertTrue(SCORING.runtime_is_complete("early_landing"))
        self.assertTrue(SCORING.runtime_is_complete("duration_complete"))
        self.assertFalse(SCORING.runtime_is_complete("interrupted"))
        self.assertTrue(SCORING.runtime_evidence_is_complete(
            "early_landing", True
        ))
        self.assertFalse(SCORING.runtime_evidence_is_complete(
            "early_landing", False
        ))

    def test_failed_transaction_cannot_earn_accuracy_or_latency_points(self):
        run = {
            "success": False,
            "rmse_m": 0.0,
            "p95_m": 0.0,
            "endpoint_m": 0.0,
            "recovery_wall_s": 0.0,
            "motion_distance_m": 0.0,
            "motion_duration_s": 0.0,
            "runtime_complete": True,
            "landed_disarmed": True,
        }
        result = SCORING.score_run(run)
        self.assertEqual(result["score_components"]["success"], 0.0)
        self.assertEqual(result["score_components"]["accuracy"], 0.0)
        self.assertEqual(result["score_components"]["latency"], 0.0)

    def test_lower_error_and_latency_receive_a_higher_score(self):
        common = {
            "success": True,
            "motion_distance_m": 0.0,
            "motion_duration_s": 0.0,
            "runtime_complete": True,
            "landed_disarmed": True,
        }
        good = SCORING.score_run({
            **common,
            "rmse_m": 0.03,
            "p95_m": 0.05,
            "endpoint_m": 0.03,
            "recovery_wall_s": 5.0,
        })
        weak = SCORING.score_run({
            **common,
            "rmse_m": 0.20,
            "p95_m": 0.30,
            "endpoint_m": 0.25,
            "recovery_wall_s": 35.0,
        })
        self.assertGreater(good["score"], weak["score"])

    def test_backend_rollbacks_reduce_score_and_block_deployment(self):
        run = {
            "success": True,
            "accuracy_passed": True,
            "rmse_m": 0.03,
            "p95_m": 0.05,
            "endpoint_m": 0.03,
            "recovery_wall_s": 5.0,
            "motion_distance_m": 0.0,
            "motion_duration_s": 0.0,
            "runtime_complete": True,
            "landed_disarmed": True,
            "optimization_errors": 0,
            "native_worker_errors": 0,
            "optimization_rollbacks": 2,
            "native_consumed_without_state_commit": 2,
            "whole_run_backend_integrity_clean": False,
        }
        result = SCORING.score_run(run)

        self.assertEqual(result["backend_integrity_events"], 2)
        self.assertEqual(
            result["score_components"]["backend_integrity_penalty"], -6.0
        )
        self.assertFalse(result["deployment_eligible"])

    def test_clean_relocalization_can_be_candidate_before_system_deployment(self):
        run = {
            "success": True,
            "accuracy_passed": True,
            "rmse_m": 0.03,
            "p95_m": 0.05,
            "endpoint_m": 0.03,
            "recovery_wall_s": 5.0,
            "motion_distance_m": 0.0,
            "motion_duration_s": 0.0,
            "runtime_complete": True,
            "landed_disarmed": True,
            "optimization_errors": 0,
            "native_worker_errors": 0,
            "optimization_rollbacks": 0,
            "native_consumed_without_state_commit": 0,
            "whole_run_backend_integrity_clean": False,
        }
        result = SCORING.score_run(run)

        self.assertTrue(result["relocalization_candidate_eligible"])
        self.assertFalse(result["deployment_eligible"])

    def test_native_queue_discard_reduces_score_and_blocks_deployment(self):
        run = {
            "success": True,
            "accuracy_passed": True,
            "rmse_m": 0.03,
            "p95_m": 0.05,
            "endpoint_m": 0.03,
            "recovery_wall_s": 5.0,
            "motion_distance_m": 0.0,
            "motion_duration_s": 0.0,
            "runtime_complete": True,
            "landed_disarmed": True,
            "optimization_errors": 0,
            "native_worker_errors": 0,
            "optimization_rollbacks": 0,
            "native_consumed_without_state_commit": 0,
            "native_worker_queue_discarded": 1,
            "native_worker_queue_overflow": 0,
            "whole_run_backend_integrity_clean": False,
        }
        result = SCORING.score_run(run)

        self.assertEqual(result["backend_integrity_events"], 1)
        self.assertEqual(
            result["score_components"]["backend_integrity_penalty"], -3.0
        )
        self.assertFalse(result["relocalization_candidate_eligible"])

    def test_post_relocalization_integrity_deltas_take_precedence(self):
        counts, evidence = SCORING.backend_integrity_counts({
            "optimization_errors": "4",
            "optimization_rollbacks": "8",
            "native_consumed_without_state_commit": "9",
            "native_worker_errors": "2",
            "relocalization_post_reset_optimization_errors": "0",
            "relocalization_post_reset_optimization_rollbacks": "1",
            "relocalization_post_reset_native_without_commit": "1",
            "relocalization_post_reset_native_worker_errors": "0",
            "relocalization_post_reset_native_queue_discarded": "2",
            "relocalization_post_reset_native_queue_overflow": "0",
        })

        self.assertEqual(evidence, "post_relocalization_delta")
        self.assertEqual(counts["optimization_rollbacks"], 1)
        self.assertEqual(counts["native_consumed_without_state_commit"], 1)
        self.assertEqual(counts["native_worker_queue_discarded"], 2)

    def test_single_run_is_screening_evidence(self):
        summaries = SCORING.summarize([{
            "scenario": "nominal",
            "label": "yaw",
            "success": True,
            "score": 80.0,
            "candidate_id": 6,
            "runtime_complete": True,
            "landed_disarmed": True,
        }])
        self.assertEqual(summaries[0]["evidence_confidence"], "screening_only")
        self.assertTrue(math.isclose(summaries[0]["success_rate"], 1.0))


if __name__ == "__main__":
    unittest.main()
