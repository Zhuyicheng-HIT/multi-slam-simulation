import importlib.util
from pathlib import Path

from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
from uf_interfaces.msg import SchedulerState


TOOL = Path(__file__).resolve().parents[4] / "tools/replay_scheduler_mask.py"
SPEC = importlib.util.spec_from_file_location("replay_scheduler_mask", TOOL)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def scheduler_message():
    message = SchedulerState()
    message.health_state = "DEGRADED"
    message.modality_names = ["lidar", "gnss", "imu", "optical_flow", "vision"]
    message.degradation_scores = [0.2, 0.3, 0.0, 0.1, 0.4]
    message.reliability_weights = [0.8, 0.7, 1.0, 0.9, 0.6]
    message.covariance_inflation = [1.2, 1.3, 1.0, 1.1, 1.4]
    message.factor_enabled = [True, True, True, True, True]
    message.reasons = ["lidar_reason", "", "", "", "vision_reason"]
    message.capability_names = ["horizontal_position"]
    message.capability_support = [0.6]
    message.capability_observable = [True]
    message.estimator_support = 0.6
    return message


def test_mask_changes_only_selected_modality_admission_fields():
    source = scheduler_message()
    result = MODULE.mask_scheduler_state(source, {"lidar"})
    lidar = result.modality_names.index("lidar")
    assert result.degradation_scores[lidar] == 1.0
    assert result.reliability_weights[lidar] == 0.0
    assert result.covariance_inflation[lidar] == 1.0e6
    assert result.factor_enabled[lidar] is False
    assert result.reasons[lidar] == "lidar_reason,replay_ablation_disabled"
    for name in ("gnss", "imu", "optical_flow", "vision"):
        index = result.modality_names.index(name)
        assert result.degradation_scores[index] == source.degradation_scores[index]
        assert result.reliability_weights[index] == source.reliability_weights[index]
        assert result.covariance_inflation[index] == source.covariance_inflation[index]
        assert result.factor_enabled[index] == source.factor_enabled[index]
        assert result.reasons[index] == source.reasons[index]
    assert result.health_state == source.health_state
    assert result.capability_names == source.capability_names
    assert result.capability_support == source.capability_support
    assert result.capability_observable == source.capability_observable
    assert result.estimator_support == source.estimator_support


def test_dual_mask_does_not_mutate_recorded_message():
    source = scheduler_message()
    original_enabled = list(source.factor_enabled)
    result = MODULE.mask_scheduler_state(source, {"lidar", "gnss"})
    assert source.factor_enabled == original_enabled
    assert result.factor_enabled[:2] == [False, False]
    assert result.factor_enabled[2:] == original_enabled[2:]


def test_mask_qos_matches_bag_input_and_backend_output_contract():
    assert qos_profile_sensor_data.reliability == ReliabilityPolicy.BEST_EFFORT
    assert MODULE.SCHEDULER_OUTPUT_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert MODULE.SCHEDULER_OUTPUT_QOS.depth == 20
