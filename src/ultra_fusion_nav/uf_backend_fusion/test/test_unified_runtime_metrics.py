import importlib.util
import math
from pathlib import Path

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from uf_interfaces.msg import FaultState, LidarCalibrationMotion, ReliabilityScore


REPO_ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location(
    "unified_runtime_metrics", REPO_ROOT / "tools" / "unified_runtime_metrics.py"
)
METRICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(METRICS)


def odometry(stamp_ns, position, yaw, velocity_body=(0.0, 0.0, 0.0)):
    message = Odometry()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        int(stamp_ns), 1_000_000_000
    )
    message.pose.pose.position.x = float(position[0])
    message.pose.pose.position.y = float(position[1])
    message.pose.pose.position.z = float(position[2])
    message.pose.pose.orientation.z = math.sin(0.5 * yaw)
    message.pose.pose.orientation.w = math.cos(0.5 * yaw)
    message.twist.twist.linear.x = float(velocity_body[0])
    message.twist.twist.linear.y = float(velocity_body[1])
    message.twist.twist.linear.z = float(velocity_body[2])
    return message


def test_epoch_continuity_uses_exact_post_sample_and_wraps_yaw():
    stats = METRICS.StreamStats()
    stats.add(odometry(9_900_000_000, (0.0, 0.0, 0.0), math.radians(179.0)))
    stats.add(odometry(10_000_000_000, (0.02, 0.0, 0.0), math.radians(-179.0)))

    result = stats.epoch_continuity(10_000_000_000)

    assert result["available"]
    assert math.isclose(result["bracket_gap_s"], 0.1)
    assert math.isclose(result["position_step_m"], 0.02)
    assert math.isclose(result["yaw_step_rad"], math.radians(2.0), abs_tol=1.0e-12)
    assert result["after_stamp_s"] == 10.0


def test_epoch_continuity_separates_motion_from_coordinate_correction():
    stats = METRICS.StreamStats()
    yaw = 0.5 * math.pi
    stats.add(odometry(
        9_900_000_000, (0.0, 0.0, 0.0), yaw, velocity_body=(1.0, 0.0, 0.0)
    ))
    stats.add(odometry(
        10_000_000_000, (0.0, 0.30, 0.0), yaw, velocity_body=(1.0, 0.0, 0.0)
    ))

    result = stats.epoch_continuity(10_000_000_000)

    assert math.isclose(result["position_step_m"], 0.30)
    assert math.isclose(result["constant_velocity_position_residual_m"], 0.20)
    assert math.isclose(result["linear_velocity_step_mps"], 0.0)


def test_epoch_continuity_sorts_out_of_order_samples_and_excludes_epoch_step():
    stats = METRICS.StreamStats()
    stats.add(odometry(10_100_000_000, (0.4, 0.0, 0.0), 0.0))
    stats.add(odometry(9_800_000_000, (0.0, 0.0, 0.0), 0.0))
    stats.add(odometry(9_900_000_000, (0.1, 0.0, 0.0), 0.0))
    stats.add(odometry(10_000_000_000, (0.3, 0.0, 0.0), 0.0))

    result = stats.epoch_continuity(10_000_000_000)

    assert stats.regressions == 1
    assert math.isclose(result["position_step_m"], 0.2)
    assert math.isclose(result["neighbor_max_step_m"], 0.1)


def test_epoch_continuity_requires_both_sides():
    stats = METRICS.StreamStats()
    stats.add(odometry(10_000_000_000, (0.0, 0.0, 0.0), 0.0))

    result = stats.epoch_continuity(10_000_000_000)

    assert not result["available"]
    assert result["reason"] == "missing_bracketing_samples"


def test_stream_stats_reports_observer_gap_and_source_age():
    stats = METRICS.StreamStats()
    stats.add(
        odometry(10_000_000_000, (0.0, 0.0, 0.0), 0.0),
        10_100_000_000,
    )
    stats.add(
        odometry(10_100_000_000, (0.1, 0.0, 0.0), 0.0),
        10_200_000_000,
    )
    stats.add(
        odometry(11_100_000_000, (0.2, 0.0, 0.0), 0.0),
        10_300_000_000,
    )

    result = stats.report()

    assert result["max_gap_s"] == 1.0
    assert math.isclose(result["observer_ros_max_gap_s"], 0.1)
    assert result["source_age_s"]["median"] == 0.1
    assert result["future_stamp_over_0_05_s"] == 1
    assert result["stale_stamp_over_0_25_s"] == 0


def test_named_counts_parse_backend_cumulative_integrity_events():
    assert METRICS.parse_named_counts(
        "ok:68,ill_conditioned_latest_information:126"
    ) == {
        "ok": 68,
        "ill_conditioned_latest_information": 126,
    }
    assert METRICS.parse_named_counts("none") == {}


def test_named_counts_reject_malformed_or_negative_entries():
    for value in ("missing_count", "ok:-1", "ok:not_an_integer"):
        try:
            METRICS.parse_named_counts(value)
        except ValueError:
            continue
        raise AssertionError(f"expected ValueError for {value!r}")


def test_scheduler_clock_domain_waits_for_observer_clock_initialization():
    assert METRICS.scheduler_clock_domain_error_s(85_000_000_000, 0) is None
    assert math.isinf(
        METRICS.scheduler_clock_domain_error_s(0, 85_000_000_000)
    )
    assert math.isclose(
        METRICS.scheduler_clock_domain_error_s(
            85_100_000_000, 85_000_000_000
        ),
        0.1,
    )


def stamped_message(message_type, stamp_ns):
    message = message_type()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        int(stamp_ns), 1_000_000_000
    )
    return message


def test_header_stream_stats_separates_source_gap_from_observer_gap_and_clock_age():
    stats = METRICS.HeaderStreamStats()
    stats.add(stamped_message(NavSatFix, 10_000_000_000), 10_100_000_000, 0.0)
    stats.add(stamped_message(NavSatFix, 10_100_000_000), 10_200_000_000, 0.1)
    stats.add(stamped_message(NavSatFix, 11_100_000_000), 10_300_000_000, 0.2)

    result = stats.report()

    assert result["source_max_gap_s"] == 1.0
    assert result["source_gaps_over_0_5_s"] == 1
    assert math.isclose(result["observer_ros_max_gap_s"], 0.1)
    assert result["source_age_s"]["median"] == 0.1
    assert result["future_stamp_over_0_05_s"] == 1
    assert result["stale_stamp_over_0_25_s"] == 0


def test_header_stream_stats_flags_future_and_clock_domain_mismatch():
    stats = METRICS.HeaderStreamStats()
    stats.add(stamped_message(NavSatFix, 20_000_000_000), 10_000_000_000)
    stats.add(stamped_message(NavSatFix, 10_000_000_000), 10_100_000_000)

    result = stats.report()

    assert result["future_stamp_over_0_05_s"] == 1
    assert result["clock_domain_mismatch_over_1_s"] == 1
    assert result["stamp_regressions"] == 1


def test_reliability_and_fault_stats_keep_validity_and_repair_evidence():
    score = stamped_message(ReliabilityScore, 1_000_000_000)
    score.valid = True
    score.degradation_score = 0.2
    score.reliability_weight = 0.8
    score.observation_count = 3
    score.reasons = ["ok"]
    score_stats = METRICS.ReliabilityScoreStats()
    score_stats.add(score, 1_100_000_000)
    score.valid = False
    score.degradation_score = 1.0
    score.reliability_weight = 0.0
    score.reasons = ["stale_or_invalid"]
    score_stats.add(score, 2_000_000_000)
    score_report = score_stats.report()

    assert score_report["valid_ratio"] == 0.5
    assert score_report["reasons"] == {"ok": 1, "stale_or_invalid": 1}

    fault = stamped_message(FaultState, 2_000_000_000)
    fault.modality = "gnss"
    fault.fault_type = "none"
    fault.timestamp_repairs = 2
    fault.timestamp_repaired = True
    fault.affected_messages = 4
    fault_stats = METRICS.FaultStateStats()
    fault_stats.add(fault)
    fault_report = fault_stats.report()

    assert fault_report["max_timestamp_repairs"] == 2
    assert fault_report["timestamp_repaired_events"] == 1
    assert fault_report["max_affected_messages"] == 4


def test_scheduler_phase_summary_keeps_factor_decisions_separate():
    timeline = [
        {
            "mission_phase": "route_active",
            "stamp_s": 10.0,
            "health_state": "NORMAL",
            "estimator_support": 0.8,
            "factor_enabled": {"gnss": True, "optical_flow": True},
            "degradation_scores": {"gnss": 0.1, "optical_flow": 0.2},
        },
        {
            "mission_phase": "route_active",
            "stamp_s": 10.5,
            "health_state": "DEGRADED",
            "estimator_support": 0.6,
            "factor_enabled": {"gnss": True, "optical_flow": False},
            "degradation_scores": {"gnss": 0.1, "optical_flow": 1.0},
        },
        {
            "mission_phase": "landing",
            "stamp_s": 11.0,
            "health_state": "DEGRADED",
            "estimator_support": 0.5,
            "factor_enabled": {"gnss": True, "optical_flow": False},
            "degradation_scores": {"gnss": 0.1, "optical_flow": 1.0},
        },
    ]

    result = METRICS.scheduler_phase_summary(timeline)

    route = result["route_active"]
    assert route["sample_count"] == 2
    assert route["health_states"] == {"NORMAL": 1, "DEGRADED": 1}
    assert route["factor_enabled_ratio"]["gnss"] == 1.0
    assert route["factor_enabled_ratio"]["optical_flow"] == 0.5
    assert math.isclose(route["degradation_score_mean"]["optical_flow"], 0.6)
    assert math.isclose(route["estimator_support_mean"], 0.7)
    assert result["landing"]["sample_count"] == 1


def test_calibration_diagnostic_sample_preserves_independent_lock_gates():
    values = {
        "calibration_reason": "candidate_pending",
        "calibration_motion_reason": "ok",
        "calibration_mode": "shadow",
        "calibration_motion_received": "41",
        "calibration_motion_rejected": "9",
        "calibration_pair_count": "13",
        "calibration_time_candidate_valid": "True",
        "calibration_time_candidate_offset_s": "0.015",
        "calibration_time_candidate_pairs": "12",
        "calibration_time_candidate_reason": "ok",
        "calibration_excitation_eigenvalues": "0.001,0.02,0.1",
        "calibration_excitation_ratio": "0.01",
        "calibration_accumulated_rotation_rad": "0.42",
        "calibration_rotation_inlier_ratio": "0.82",
        "calibration_time_locked": "true",
        "calibration_rotation_locked": "false",
        "calibration_locked": "False",
    }

    sample = METRICS.calibration_diagnostic_sample(
        values, 12.5, "calibration_excitation"
    )

    assert sample["stamp_s"] == 12.5
    assert sample["mission_phase"] == "calibration_excitation"
    assert sample["calibration_pair_count"] == 13
    assert sample["calibration_time_candidate_offset_s"] == 0.015
    assert sample["calibration_excitation_eigenvalues"] == [0.001, 0.02, 0.1]
    assert sample["calibration_time_locked"] is True
    assert sample["calibration_rotation_locked"] is False
    assert sample["calibration_locked"] is False


def test_calibration_motion_stats_keep_raw_rotation_and_quality_by_phase():
    stats = METRICS.CalibrationMotionStats()
    motion = LidarCalibrationMotion()
    motion.header.stamp.sec = 10
    motion.start_stamp.sec = 9
    motion.start_stamp.nanosec = 600_000_000
    motion.accepted = True
    motion.converged = True
    motion.reason = "accepted"
    motion.relative_rotation.z = math.sin(0.05)
    motion.relative_rotation.w = math.cos(0.05)
    motion.relative_translation.x = 0.2
    motion.inlier_ratio = 0.8
    motion.residual_rms_m = 0.15

    stats.add(motion, "calibration_excitation")
    report = stats.report()
    phase = report["phase_summary"]["calibration_excitation"]

    assert report["accepted"] == 1
    assert phase["accepted_ratio"] == 1.0
    assert math.isclose(phase["accepted_interval_s"]["median"], 0.4)
    assert math.isclose(phase["accepted_rotation_angle_rad"]["median"], 0.1)
    assert math.isclose(phase["accepted_translation_m"]["median"], 0.2)
    assert math.isclose(
        phase["accepted_quality_weight"]["median"], 0.8 * math.exp(-1.0)
    )
