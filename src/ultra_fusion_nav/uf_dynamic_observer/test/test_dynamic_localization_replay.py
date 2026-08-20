import importlib.util
from pathlib import Path


def load_analyzer():
    source = (
        Path(__file__).resolve().parents[1]
        / "tools"
        / "analyze_dynamic_localization_replay.py"
    )
    spec = importlib.util.spec_from_file_location("dyn_loc_007_analyzer", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_recovery_requires_consecutive_healthy_estimates():
    module = load_analyzer()
    value = module.recovery_time_s(
        [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
        [0.08, 0.02, 0.07, 0.02, 0.019, 0.018],
        0.03,
        consecutive=3,
    )
    assert value == 0.3


def test_recovery_does_not_invent_success():
    module = load_analyzer()
    assert module.recovery_time_s([0.0, 0.1], [0.04, 0.02], 0.03, consecutive=2) is None


def test_dynamic_scenarios_exclude_static_control_and_keep_occlusion_split():
    module = load_analyzer()
    assert "static_baseline" not in module.PRIMARY_SCENARIOS
    assert module.OCCLUSION_SCENARIOS == [
        "c1_persistent_occlusion",
        "c2_same_view_reobservation",
        "c3_natural_multiview_reobservation",
    ]
