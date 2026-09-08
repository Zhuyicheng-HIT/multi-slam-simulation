from pathlib import Path
import importlib.util
import subprocess


REPO = Path(__file__).resolve().parents[3]
TOOLS = REPO / "tools"


SPEC = importlib.util.spec_from_file_location(
    "verify_competition_bag", TOOLS / "verify_competition_bag.py"
)
VERIFIER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VERIFIER)


def run_config(mode: str) -> dict[str, str]:
    result = subprocess.run(
        ["bash", str(TOOLS / "run_competition_trial.sh"), "--print-config", mode],
        check=True,
        capture_output=True,
        text=True,
    )
    return dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)


def test_four_public_operator_commands_delegate_to_one_implementation():
    wrappers = {
        "competition_metric1.sh": "metric1",
        "competition_metric2_full.sh": "metric2-full",
        "competition_metric2_no_camera.sh": "metric2-no-camera",
        "competition_metric3.sh": "metric3-five-source",
        "competition_metric3_four_source.sh": "metric3-four-source",
        "competition_metric3_five_source.sh": "metric3-five-source",
    }
    for name, mode in wrappers.items():
        text = (TOOLS / name).read_text(encoding="utf-8")
        assert "run_competition_trial.sh" in text
        assert mode in text


def test_metric1_is_fixed_before_flight_to_three_source_medium_lidar_mode():
    config = run_config("metric1")
    assert config["enable_gnss"] == "false"
    assert config["enable_vision"] == "false"
    assert config["active_modalities"] == "lidar,imu,optical_flow"
    assert config["robustness_profile"] == "lidar_medium"
    assert config["robustness_channels"] == "native_lidar"
    assert config["dynamic_observer"] == "false"


def test_metric2_has_separate_full_and_no_camera_modes():
    full = run_config("metric2-full")
    no_camera = run_config("metric2-no-camera")
    assert full["enable_gnss"] == "true"
    assert full["enable_vision"] == "true"
    assert full["dynamic_observer"] == "true"
    assert no_camera["enable_gnss"] == "true"
    assert no_camera["enable_vision"] == "false"
    assert no_camera["dynamic_observer"] == "true"
    assert full["robustness_profile"] == no_camera["robustness_profile"] == "nominal"


def test_metric3_has_four_and_five_source_manual_relocalization_modes():
    four = run_config("metric3-four-source")
    five = run_config("metric3-five-source")
    assert four["enable_gnss"] == five["enable_gnss"] == "true"
    assert four["enable_vision"] == "false"
    assert four["active_modalities"] == "lidar,gnss,imu,optical_flow"
    assert five["enable_vision"] == "true"
    assert five["active_modalities"] == "lidar,gnss,imu,optical_flow,vision"
    for config in (four, five):
        assert config["robustness_profile"] == "nominal"
        assert config["manual_relocalization"] == "true"
        assert config["require_relocalization_database"] == "true"
        assert config["dynamic_observer"] == "false"


def test_recorder_preserves_raw_evidence_events_and_clean_shutdown_contract():
    text = (TOOLS / "record_competition_bag.sh").read_text(encoding="utf-8")
    for topic in (
        "/livox/lidar",
        "/livox/imu",
        "/sensors/optical_flow/rad",
        "/sensors/rgbd/color",
        "/fusion/unified/odom",
        "/dynamic_observer/dynamic_candidates",
        "/relocalization/result",
        "/fusion/unified/epoch",
        "/competition/event",
    ):
        assert topic in text
    assert "--compression-format zstd" in text
    assert "kill -INT" in text
    assert "wait \"$recorder_pid\"" in text
    assert "metadata.yaml" in text
    assert "ros2 bag info" in text
    assert "sha256sum" in text


def test_manual_control_uses_owned_service_not_final_request_or_mavros():
    text = (TOOLS / "manual_relocalization.sh").read_text(encoding="utf-8")
    assert "/relocalization/manual_control" in text
    assert "uf_interfaces/srv/ManualRelocalization" in text
    assert "source: manual_control" in text
    assert "ManualRelocalization.Request.START" not in text
    assert "ros2 topic pub /relocalization/request" not in text
    assert "/mavros/setpoint" not in text


def test_no_flight_smoke_is_isolated_and_uses_manual_service_path():
    text = (TOOLS / "manual_relocalization_no_flight_smoke.sh").read_text(
        encoding="utf-8"
    )
    assert "ROS_DOMAIN_ID" in text
    assert "ROS_LOCALHOST_ONLY=1" in text
    assert "--request-source manual_service" in text
    assert "This does not prove real relocalization accuracy" in text


def test_frequency_and_recording_validation_tools_cover_competition_inputs():
    frequency = (TOOLS / "check_competition_frequencies.sh").read_text(encoding="utf-8")
    verifier = (TOOLS / "verify_competition_bag.py").read_text(encoding="utf-8")
    for topic in (
        "/livox/lidar",
        "/livox/imu",
        "/sensors/optical_flow/rad",
        "/sensors/rgbd/color",
        "/sensors/rgbd/depth",
    ):
        assert topic in frequency
    assert "metric1" in verifier
    assert "metric2-full" in verifier
    assert "metric2-no-camera" in verifier
    assert "metric3-four-source" in verifier
    assert "metric3-five-source" in verifier
    assert "topics_with_message_count" in verifier


def metadata(counts: dict[str, int]) -> dict:
    return {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": topic},
                    "message_count": count,
                }
                for topic, count in counts.items()
            ]
        }
    }


def test_metric1_bag_rejects_standardized_gnss_or_visual_messages():
    counts = {topic: 10 for topic in VERIFIER.MODE_REQUIRED["metric1"]}
    counts["/sensors/gnss/fix"] = 1
    counts["/sensors/rgbd/color"] = 1
    report = VERIFIER.build_report("metric1", metadata(counts))
    assert not report["valid"]
    assert report["unexpected_active_topics"] == [
        "/sensors/gnss/fix",
        "/sensors/rgbd/color",
    ]


def test_metric3_bag_requires_real_result_epoch_and_active_state_evidence():
    counts = {topic: 10 for topic in VERIFIER.MODE_REQUIRED["metric3"]}
    counts.pop("/relocalization/result")
    report = VERIFIER.build_report("metric3", metadata(counts))
    assert not report["valid"]
    assert report["missing_or_empty_required_topics"] == ["/relocalization/result"]


def test_metric3_four_source_rejects_visual_while_five_source_requires_it():
    four_counts = {
        topic: 10 for topic in VERIFIER.MODE_REQUIRED["metric3-four-source"]
    }
    four_counts["/sensors/rgbd/color"] = 1
    four_report = VERIFIER.build_report(
        "metric3-four-source", metadata(four_counts)
    )
    assert not four_report["valid"]
    assert four_report["unexpected_active_topics"] == ["/sensors/rgbd/color"]

    five_counts = {
        topic: 10 for topic in VERIFIER.MODE_REQUIRED["metric3-five-source"]
    }
    five_counts.pop("/sensors/rgbd/depth")
    five_report = VERIFIER.build_report(
        "metric3-five-source", metadata(five_counts)
    )
    assert not five_report["valid"]
    assert five_report["missing_or_empty_required_topics"] == [
        "/sensors/rgbd/depth"
    ]
