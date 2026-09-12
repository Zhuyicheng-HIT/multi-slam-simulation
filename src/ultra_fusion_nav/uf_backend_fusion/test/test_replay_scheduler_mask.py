import importlib.util
from pathlib import Path

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
    message.factor_enabled = [True] * 5
    message.reasons = ["lidar_reason", "", "", "", "vision_reason"]
    message.capability_names = ["horizontal_position"]
    message.capability_support = [0.6]
    message.capability_observable = [True]
    message.estimator_support = 0.6
    return message


def test_mask_changes_only_selected_modality():
    source = scheduler_message()
    result = MODULE.mask_scheduler_state(source, {"lidar"})
    index = result.modality_names.index("lidar")
    assert result.factor_enabled[index] is False
    assert result.reliability_weights[index] == 0.0
    assert result.covariance_inflation[index] == 1.0e6
    assert result.reasons[index] == "lidar_reason,replay_ablation_disabled"
    gnss = result.modality_names.index("gnss")
    assert result.factor_enabled[gnss] is True
    assert result.reliability_weights[gnss] == source.reliability_weights[gnss]


def test_mask_does_not_mutate_source():
    source = scheduler_message()
    original = list(source.factor_enabled)
    MODULE.mask_scheduler_state(source, {"lidar", "gnss"})
    assert source.factor_enabled == original
