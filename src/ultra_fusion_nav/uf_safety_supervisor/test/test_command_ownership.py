from pathlib import Path


def test_flight_command_arbiter_is_only_direct_automatic_setpoint_publisher():
    repository = Path(__file__).resolve().parents[4]
    owners = []
    for path in (repository / "src").rglob("*"):
        if (
            path.suffix not in {".py", ".cpp", ".hpp"}
            or "/test/" in path.as_posix()
            or "/tools/" in path.as_posix()
        ):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        if '"/mavros/setpoint_position/local"' in text or "'/mavros/setpoint_position/local'" in text:
            owners.append(path.relative_to(repository).as_posix())
    assert owners == [
        "src/ultra_fusion_nav/uf_safety_supervisor/src/flight_command_arbiter.cpp"
    ]


def test_dynamic_clean_topics_are_not_obstacle_monitor_inputs():
    source = (
        Path(__file__).resolve().parents[1] / "src" / "raw_obstacle_safety_monitor.cpp"
    ).read_text(encoding="utf-8")
    assert '"/livox/lidar"' in source
    assert "/livox/lidar_clean" not in source
    assert "/dynamic_observer/static" not in source


def test_local_planner_uses_raw_lidar_and_never_owns_mavros():
    source = (
        Path(__file__).resolve().parents[1] / "src" / "local_avoidance_planner.cpp"
    ).read_text(encoding="utf-8")
    assert '"/livox/lidar"' in source
    assert "/livox/lidar_clean" not in source
    assert "/dynamic_observer/static" not in source
    assert "/mavros/setpoint_position/local" not in source
    assert '"/autonomy/intent/planner/pose"' in source
    assert '"/autonomy/candidate_path"' in source


def test_fcu_heartbeat_absence_is_fail_closed():
    source = (
        Path(__file__).resolve().parents[1] / "src" / "flight_command_arbiter.cpp"
    ).read_text(encoding="utf-8")
    assert "!fcu_received_ || !fcu_connected_" in source


def test_automatic_route_entrypoints_start_the_single_owner_safety_slice():
    repository = Path(__file__).resolve().parents[4]
    scripts = repository / "src" / "multi_slam_uav_sim" / "scripts"
    for name in (
        "run_apm_sensor_stack.sh",
        "run_rectangle_state_machine.sh",
        "run_s_curve_state_machine.sh",
        "run_pr6_d435i_visual_headless.sh",
    ):
        text = (scripts / name).read_text(encoding="utf-8")
        assert "safety_slice_start" in text, name

    helper = (scripts / "safety_slice_process.sh").read_text(encoding="utf-8")
    assert "unknown automatic setpoint publisher(s) already exist" in helper
    assert "raw_lidar_topic" in helper
