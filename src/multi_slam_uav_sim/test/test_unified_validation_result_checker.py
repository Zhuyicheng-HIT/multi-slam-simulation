import importlib.util
from pathlib import Path
import unittest


TOOL_PATH = Path(__file__).resolve().parents[3] / "tools" / "check_unified_validation_result.py"
SPEC = importlib.util.spec_from_file_location("check_unified_validation_result", TOOL_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _valid_inputs():
    accuracy = {
        "acceptance": {"passed": True},
        "initial_alignment": {"future_trajectory_used": False},
        "truth_used_by_estimator": False,
        "matched_samples": 1200,
        "motion_samples": 300,
        "causal_ate": {
            "three_dimensional": {"rmse_m": 0.05, "p95_m": 0.08, "max_m": 0.12},
            "endpoint_error_m": {"norm": 0.03},
            "horizontal": {"rmse_m": 0.03},
            "vertical": {"rmse_m": 0.04},
        },
    }
    stream = {
        "count": 1200,
        "source_stamp_rate_hz": 10.0,
        "max_gap_s": 0.12,
        "source_age_s": {"p95": 0.08},
        "stale_stamp_over_0_25_s": 0,
        "stamp_duplicates": 0,
        "stamp_regressions": 0,
        "zero_stamps": 0,
        "max_displacement_from_first_m": 3.0,
    }
    backend = {
        "optimization_errors": "0",
        "optimization_rollbacks": "0",
        "native_worker_errors": "0",
        "native_worker_queue_overflow": "0",
        "native_worker_queue_discarded": "0",
        "native_consumed_without_state_commit": "0",
        "lidar_factors": "100",
        "imu_factors": "99",
        "gnss_factors": "20",
        "flow_factors": "25",
        "visual_factors": "12",
        "covariance_source": "window_marginal",
        "calibration_time_locked": "True",
        "calibration_time_offset_s": "0.035",
        "calibration_mode": "time_apply",
    }
    external = dict(stream)
    external["source_stamp_rate_hz"] = 20.0
    runtime = {
        "termination_reason": "duration_complete",
        "sim_duration_s": 170.0,
        "graph_contract_violations": None,
        "streams": {"unified_odom": stream, "externalnav_out": external},
        "backend_latest": backend,
        "externalnav_diagnostic_reasons": {"publishing": 170},
    }
    route = "\n".join([
        "Mission phase: preflight",
        "Mission phase: post_takeoff_hold",
        "Mission phase: route_active",
        "waypoint 1/4: (0,0,3) -> (2,0,3)",
        "waypoint 2/4: (2,0,3) -> (2,1,3)",
        "waypoint 3/4: (2,1,3) -> (0,1,3)",
        "waypoint 4/4: (0,1,3) -> (0,0,3)",
        "Mission phase: landing",
        "LAND completed and FCU disarm confirmed.",
    ])
    mavros = "FCU: EKF3 IMU0 is using external nav data"
    return accuracy, runtime, route, mavros, "normal SITL shutdown"


class UnifiedValidationResultCheckerTest(unittest.TestCase):
    def test_legacy_backend_without_lidar_disabled_counts_all_lidar_factors(self):
        self.assertEqual(
            MODULE._accepted_factor_counts(
                {
                    "lidar_factors": "7",
                    "imu_factors": "6",
                    "gnss_factors": "2",
                    "flow_factors": "3",
                    "visual_factors": "1",
                }
            )["lidar"],
            7,
        )

    def test_accepts_complete_externalnav_run(self):
        report = MODULE.evaluate_validation(
            *_valid_inputs(),
            require_external_nav=True,
            require_time_lock=True,
            require_time_applied=True,
        )
        self.assertTrue(report["passed"])
        self.assertEqual(report["failed_gates"], [])

    def test_accepts_landed_phase_runtime_stop(self):
        accuracy, runtime, route, mavros, sitl = _valid_inputs()
        runtime["termination_reason"] = "mission_phase:landed"
        runtime["sim_duration_s"] = 156.0

        report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            minimum_sim_duration_s=120.0,
        )

        self.assertTrue(report["passed"])
        self.assertEqual(report["failed_gates"], [])

    def test_rejects_landing_phase_runtime_stop_before_disarm(self):
        accuracy, runtime, route, mavros, sitl = _valid_inputs()
        runtime["termination_reason"] = "mission_phase:landing"
        runtime["sim_duration_s"] = 156.0

        report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            minimum_sim_duration_s=120.0,
        )

        self.assertFalse(report["passed"])
        self.assertIn("runtime_completed_requested_duration", report["failed_gates"])

    def test_rejects_stationary_false_success(self):
        accuracy, runtime, route, mavros, sitl = _valid_inputs()
        runtime["streams"]["unified_odom"]["max_displacement_from_first_m"] = 0.02
        report = MODULE.evaluate_validation(accuracy, runtime, route, mavros, sitl)
        self.assertFalse(report["passed"])
        self.assertIn("vehicle_executed_nontrivial_motion", report["failed_gates"])

    def test_accepts_early_landing_after_confirmed_disarm(self):
        accuracy, runtime, route, mavros, sitl = _valid_inputs()
        runtime["termination_reason"] = "early_landing"
        runtime["sim_duration_s"] = 80.0
        report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            minimum_sim_duration_s=0.0,
        )

        self.assertTrue(report["passed"])

    def test_rejects_early_landing_without_confirmed_disarm(self):
        accuracy, runtime, route, mavros, sitl = _valid_inputs()
        runtime["termination_reason"] = "early_landing"
        route = route.replace(
            "LAND completed and FCU disarm confirmed.",
            "land command sent",
        )
        report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            minimum_sim_duration_s=0.0,
        )

        self.assertFalse(report["passed"])
        self.assertIn(
            "runtime_completed_requested_duration", report["failed_gates"]
        )

    def test_rejects_missing_ekf3_consumption_and_landing(self):
        accuracy, runtime, route, _, sitl = _valid_inputs()
        route = route.replace("LAND completed and FCU disarm confirmed.", "land command sent")
        report = MODULE.evaluate_validation(
            accuracy, runtime, route, "no external navigation status", sitl,
            require_external_nav=True,
        )
        self.assertFalse(report["passed"])
        self.assertIn("ekf3_confirmed_external_nav_consumption", report["failed_gates"])
        self.assertIn("landing_and_disarm_confirmed", report["failed_gates"])

    def test_rejects_optimizer_error_and_unlocked_time(self):
        accuracy, runtime, route, mavros, sitl = _valid_inputs()
        runtime["backend_latest"]["optimization_errors"] = "1"
        runtime["backend_latest"]["calibration_time_locked"] = "False"
        report = MODULE.evaluate_validation(
            accuracy, runtime, route, mavros, sitl,
            require_time_lock=True,
        )
        self.assertFalse(report["passed"])
        self.assertIn("backend_optimization_errors_zero", report["failed_gates"])
        self.assertIn("online_time_calibration_locked", report["failed_gates"])

    def test_visual_factor_gate_is_opt_in(self):
        accuracy, runtime, route, mavros, sitl = _valid_inputs()
        runtime["backend_latest"]["visual_factors"] = "0"
        baseline = MODULE.evaluate_validation(accuracy, runtime, route, mavros, sitl)
        visual = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            require_visual_factors=True,
        )
        self.assertTrue(baseline["passed"])
        self.assertFalse(visual["passed"])
        self.assertIn("backend_visual_factors_active", visual["failed_gates"])

    def test_accepts_calibration_mission_and_requires_both_time_locks(self):
        accuracy, runtime, _, mavros, sitl = _valid_inputs()
        runtime["backend_latest"]["visual_time_offset_locked"] = "True"
        runtime["backend_latest"]["visual_time_offset_s"] = "-0.018"
        route = "\n".join([
            "Mission phase: preflight",
            "Mission phase: post_takeoff_hold",
            "Mission phase: calibration_excitation",
            "Mission phase: calibration_complete",
            "Mission phase: landing",
            "LAND completed and FCU disarm confirmed.",
        ])

        report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            require_external_nav=True,
            require_time_lock=True,
            require_visual_time_lock=True,
            require_visual_factors=True,
            mission_profile="calibration",
            expected_waypoints=0,
        )

        self.assertTrue(report["passed"])
        self.assertEqual(
            report["acceptance_basis"],
            "strict_unified_calibration_end_to_end",
        )

    def test_calibration_mission_rejects_missing_visual_time_lock(self):
        accuracy, runtime, _, mavros, sitl = _valid_inputs()
        runtime["backend_latest"]["visual_time_offset_locked"] = "False"
        runtime["backend_latest"]["visual_time_offset_s"] = "0.0"
        route = "\n".join([
            "Mission phase: preflight",
            "Mission phase: post_takeoff_hold",
            "Mission phase: calibration_excitation",
            "Mission phase: calibration_complete",
            "Mission phase: landing",
            "LAND completed and FCU disarm confirmed.",
        ])

        report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            require_visual_time_lock=True,
            mission_profile="calibration",
            expected_waypoints=0,
        )

        self.assertFalse(report["passed"])
        self.assertIn("visual_time_calibration_locked", report["failed_gates"])

    def test_accepts_automatic_loop_closure_on_complete_figure_eight(self):
        accuracy, runtime, _, mavros, sitl = _valid_inputs()
        runtime["automatic_loop_searches"] = 3
        runtime["automatic_loop_successes"] = 1
        runtime["relocalization_timeline"] = [{
            "mission_phase": "route_active",
            "accepted": True,
            "reason": "automatic_loop_candidate_accepted_awaiting_backend_epoch",
        }]
        runtime["fusion_epoch_applied"] = 1
        runtime["backend_latest"]["relocalization_resets"] = "1"
        runtime["fusion_epoch_continuity"] = [{
            "streams": {
                "unified_odom": {
                    "available": True,
                    "position_step_m": 0.08,
                    "yaw_step_rad": 0.02,
                }
            }
        }]
        route = "\n".join([
            "Mission phase: preflight",
            "Mission phase: post_takeoff_hold",
            "Mission phase: calibration_excitation",
            "Large figure-eight plan: one closed traversal, "
            "planned_path_distance=39.94m, altitude_range=4.90..9.40m, "
            "ratio_at_or_below_8m=89.2%, axis=158.0deg",
            "Mission phase: route_active",
            "large figure-eight single traversal: points=1599, "
            "distance=39.94m, feedback=unified_backend",
            *(
                f"Mission checkpoint {index}: large figure-eight single traversal"
                for index in range(1, 20)
            ),
            "closed-loop return convergence: position_error=0.04m",
            "Mission phase: landing",
            "LAND completed and FCU disarm confirmed.",
        ])

        report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            require_automatic_loop_closure=True,
            mission_profile="figure_eight",
            expected_waypoints=0,
        )

        self.assertTrue(report["passed"])
        self.assertEqual(
            report["acceptance_basis"],
            "strict_unified_figure_eight_end_to_end",
        )

    def test_low_altitude_figure_eight_uses_configured_distance_gate(self):
        accuracy, runtime, _, mavros, sitl = _valid_inputs()
        route = "\n".join([
            "Mission phase: preflight",
            "Mission phase: post_takeoff_hold",
            "Mission phase: calibration_excitation",
            "Large figure-eight plan: one closed traversal, "
            "planned_path_distance=29.08m, altitude_range=2.16..2.96m, "
            "ratio_at_or_below_8m=100.0%, axis=158.0deg",
            "Mission phase: route_active",
            "large figure-eight single traversal: points=832, "
            "distance=29.08m, feedback=unified_backend",
            *(
                f"Mission checkpoint {index}: large figure-eight single traversal"
                for index in range(1, 15)
            ),
            "Mission phase: landing",
            "LAND completed and FCU disarm confirmed.",
        ])

        default_report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            mission_profile="figure_eight",
            expected_waypoints=0,
        )
        low_altitude_report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            mission_profile="figure_eight",
            expected_waypoints=0,
            minimum_figure_eight_distance_m=29.0,
            minimum_figure_eight_checkpoints=14,
        )

        self.assertFalse(default_report["passed"])
        self.assertIn(
            "figure_eight_nontrivial_distance",
            default_report["failed_gates"],
        )
        self.assertIn(
            "figure_eight_route_completed",
            default_report["failed_gates"],
        )
        self.assertTrue(low_altitude_report["passed"])

    def test_automatic_loop_rejects_large_epoch_jump(self):
        accuracy, runtime, _, mavros, sitl = _valid_inputs()
        runtime["automatic_loop_searches"] = 1
        runtime["automatic_loop_successes"] = 1
        runtime["relocalization_timeline"] = [{
            "mission_phase": "route_active",
            "accepted": True,
            "reason": "automatic_loop_candidate_accepted_awaiting_backend_epoch",
        }]
        runtime["fusion_epoch_applied"] = 1
        runtime["backend_latest"]["relocalization_resets"] = "1"
        runtime["fusion_epoch_continuity"] = [{
            "streams": {
                "unified_odom": {
                    "available": True,
                    "position_step_m": 0.45,
                    "yaw_step_rad": 0.02,
                }
            }
        }]
        route = "\n".join([
            "Mission phase: preflight",
            "Mission phase: post_takeoff_hold",
            "Mission phase: calibration_excitation",
            "Large figure-eight plan: one closed traversal, "
            "planned_path_distance=39.94m, altitude_range=4.90..9.40m, "
            "ratio_at_or_below_8m=89.2%, axis=158.0deg",
            "Mission phase: route_active",
            "large figure-eight single traversal: feedback=unified_backend",
            *(
                f"Mission checkpoint {index}: large figure-eight single traversal"
                for index in range(1, 20)
            ),
            "closed-loop return convergence",
            "Mission phase: landing",
            "LAND completed and FCU disarm confirmed.",
        ])

        report = MODULE.evaluate_validation(
            accuracy,
            runtime,
            route,
            mavros,
            sitl,
            require_automatic_loop_closure=True,
            mission_profile="figure_eight",
            expected_waypoints=0,
        )

        self.assertFalse(report["passed"])
        self.assertIn(
            "automatic_loop_position_step_bounded", report["failed_gates"]
        )
