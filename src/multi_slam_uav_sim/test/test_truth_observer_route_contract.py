from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_ROOT = REPO_ROOT / "src" / "multi_slam_uav_sim"
ULTRA_ROOT = REPO_ROOT / "src" / "ultra_fusion_nav"
TRUTH_TOPIC = "/sim/mid360/ground_truth_odom"


class TruthObserverRouteContractTest(unittest.TestCase):
    def test_one_click_wrapper_is_control_isolated(self):
        wrapper = (
            REPO_ROOT / "tools" / "run_figure8_truth_observer_demo.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("DEMO_ROUTE_FEEDBACK_SOURCE=gazebo_truth", wrapper)
        self.assertIn("DEMO_EXTERNAL_NAV_ENABLED=0", wrapper)
        self.assertIn("DEMO_LOCALIZATION_SAFETY_ENABLED=false", wrapper)
        self.assertIn("DEMO_PERFORMANCE_PROFILING_ENABLED=1", wrapper)

    def test_route_runner_passes_explicit_truth_source(self):
        runner = (
            SIM_ROOT / "scripts" / "run_s_curve_state_machine.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('ROUTE_FEEDBACK_SOURCE=${ROUTE_FEEDBACK_SOURCE:-', runner)
        self.assertIn('-p route_feedback_source:="$ROUTE_FEEDBACK_SOURCE"', runner)
        self.assertIn('-p gazebo_truth_odom_topic:="$GAZEBO_TRUTH_ODOM_TOPIC"', runner)

    def test_s_curve_supports_isolated_fcu_local_dataset_control(self):
        runner = (
            SIM_ROOT / "scripts" / "run_s_curve_state_machine.sh"
        ).read_text(encoding="utf-8")
        controller = (
            SIM_ROOT / "multi_slam_uav_sim" / "guided_s_curve_waypoints.py"
        ).read_text(encoding="utf-8")
        validation = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("unified_backend|fcu_local|gazebo_truth", runner)
        self.assertIn('LOCALIZATION_SAFETY_ENABLED=false', runner)
        self.assertIn('self.route_feedback_source == "fcu_local"', controller)
        self.assertIn("ESTIMATOR-ONLY ROUTE ISOLATION ENABLED", controller)
        self.assertIn("s_curve:fcu_local", validation)

    def test_rectangle_runner_selects_truth_adapter(self):
        runner = (
            SIM_ROOT / "scripts" / "run_rectangle_state_machine.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("guided_truth_rectangle_waypoints", runner)
        self.assertIn('-p route_feedback_source:=gazebo_truth', runner)
        self.assertIn('-p gazebo_truth_odom_topic:="$GAZEBO_TRUTH_ODOM_TOPIC"', runner)

    def test_validation_bag_records_barometer(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("wait_rate /sim/barometer/pressure 5.0 40", runner)
        self.assertIn("/sim/barometer/pressure", runner)
        self.assertIn("/mavros/imu/static_pressure", runner)

    def test_validation_passes_requested_feedback_to_checker(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")
        self.assertIn(
            '--expected-route-feedback "$VALIDATION_ROUTE_FEEDBACK_SOURCE"',
            runner,
        )

        checker = (
            REPO_ROOT / "tools" / "check_unified_validation_result.py"
        ).read_text(encoding="utf-8")
        self.assertIn("figure_eight_uses_requested_feedback", checker)
        self.assertNotIn("figure_eight_uses_unified_feedback", checker)

    def test_straight_route_has_an_explicit_one_waypoint_contract(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('rectangle|straight)', runner)
        self.assertIn('VALIDATION_ROUTE_MODE" == "straight"', runner)
        self.assertIn("rectangle_profile_args+=(--expected-waypoints 1)", runner)

    def test_validation_separates_world_file_profile_from_gazebo_name(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("VALIDATION_GAZEBO_WORLD_NAME", runner)
        self.assertIn('WORLD_NAME="$VALIDATION_GAZEBO_WORLD_NAME"', runner)
        self.assertIn('-p world_name:="$VALIDATION_GAZEBO_WORLD_NAME"', runner)

    def test_mid360_bridges_receive_actual_gazebo_world(self):
        runner = (
            SIM_ROOT / "scripts" / "run_apm_sensor_stack.sh"
        ).read_text(encoding="utf-8")

        python_block = runner.split(
            "ros2 run multi_slam_uav_sim gz_mid360_pointcloud_bridge", 1
        )[1].split("pids+=(\"$!\")", 1)[0]
        direct_block = runner.split(
            "ros2 run mid360_sim_bridge_cpp gz_livox_bridge_node", 1
        )[1].split("pids+=(\"$!\")", 1)[0]
        for block in (python_block, direct_block):
            self.assertIn('-p gazebo_world_name:="$WORLD_NAME"', block)
            self.assertIn("-p gazebo_model:=apm_iris", block)

    def test_external_lidar_overlay_cannot_reintroduce_an_old_project(self):
        scripts = list((REPO_ROOT / "tools").glob("*.sh"))
        scripts += list((SIM_ROOT / "scripts").glob("*.sh"))
        scripts += list(
            (REPO_ROOT / "src" / "hybridfusion_map_fusion" / "scripts").glob("*.sh")
        )
        users = []
        for script in scripts:
            text = script.read_text(encoding="utf-8")
            if 'source "$LIDAR_WS/install/' not in text:
                continue
            users.append(script)
            self.assertNotIn(
                'source "$LIDAR_WS/install/setup.bash"', text, str(script)
            )
            self.assertIn(
                'source "$LIDAR_WS/install/local_setup.bash"', text, str(script)
            )
        self.assertTrue(users)

    def test_temporal_dynamic_filter_is_explicit_and_default_off(self):
        runner = (
            SIM_ROOT / "scripts" / "run_apm_sensor_stack.sh"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "TEMPORAL_DYNAMIC_FILTER_ENABLED=${TEMPORAL_DYNAMIC_FILTER_ENABLED:-false}",
            runner,
        )
        self.assertIn("livox_temporal_dynamic_filter", runner)
        self.assertIn("/livox/lidar_raw", runner)

    def test_mid360_truth_fails_closed_without_model_pose(self):
        cpp_bridge = (
            REPO_ROOT
            / "src"
            / "mid360_sim_bridge_cpp"
            / "src"
            / "gz_livox_bridge_node.cpp"
        ).read_text(encoding="utf-8")
        python_bridge = (
            SIM_ROOT / "multi_slam_uav_sim" / "gz_mid360_pointcloud_bridge.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            '"world_stats_topic", "/world/" + gazebo_world_name_ + "/stats"',
            cpp_bridge,
        )
        self.assertIn('"allow_scan_pose_truth_fallback", false', cpp_bridge)
        self.assertIn("ground_truth_unavailable_count_", cpp_bridge)
        self.assertNotIn("pose = msg.world_pose", python_bridge)

    def test_dynamic_agents_are_explicit_and_owned_by_validation(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("VALIDATION_DYNAMIC_AGENTS_ENABLED", runner)
        self.assertIn("VALIDATION_DYNAMIC_AGENTS_CONFIG", runner)
        self.assertIn("ros2 run multi_slam_uav_sim people_motion", runner)
        self.assertIn('pids+=("$dynamic_agents_pid")', runner)

    def test_large_scene_runner_preserves_landing_stop_and_body_envelope(self):
        runner = (
            REPO_ROOT / "tools" / "run_large_scene_validation.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("VALIDATION_STOP_OBSERVERS_ON_LANDING", runner)
        self.assertIn("body_envelope_m=0.50,0.50,0.10", runner)
        self.assertIn("city_dynamic_relocalization", runner)
        self.assertIn("tunnel_dynamic_relocalization", runner)

    def test_validation_stops_collectors_when_route_fails(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("collector_stop_reason=route_failed", runner)
        self.assertIn("route_terminated: stopping collectors", runner)
        self.assertIn('stop_collector "$replay_bag_pid"', runner)
        self.assertIn('stop_collector "$relocalization_trigger_pid"', runner)

    def test_validation_records_process_and_gpu_resources(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("collect_validation_resources.py", runner)

    def test_accuracy_observers_use_wall_time_without_clock_subscription(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")
        replay = (
            REPO_ROOT / "tools" / "run_full_online_backend_replay.sh"
        ).read_text(encoding="utf-8")

        for script in (runner, replay):
            blocks = script.split("external_nav_accuracy --ros-args")[1:]
            self.assertTrue(blocks)
            for block in blocks:
                self.assertIn("-p use_sim_time:=false", block[:300])

    def test_replay_metrics_monitor_keeps_compact_timeline_and_full_last_state(self):
        recorder = (
            REPO_ROOT / "tools" / "record_backend_replay_metrics.py"
        ).read_text(encoding="utf-8")
        replay = (
            REPO_ROOT / "tools" / "run_full_online_backend_replay.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("SAMPLE_VALUE_KEYS", recorder)
        self.assertIn("self.last_values_raw = raw_values", recorder)
        self.assertIn("SAMPLE_VALUE_KEY_SET = frozenset(SAMPLE_VALUE_KEYS)", recorder)
        self.assertIn("if item.key in SAMPLE_VALUE_KEY_SET", recorder)
        self.assertIn("for key, value in node.last_values_raw.items()", recorder)
        self.assertNotIn('cat "$OUTPUT_DIR/replay_metrics.json"', replay)

    def test_replay_can_regenerate_current_reliability_decisions(self):
        replay = (
            REPO_ROOT / "tools" / "run_full_online_backend_replay.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("REGENERATE_RELIABILITY_STACK", replay)
        self.assertIn("reliability_monitor", replay)
        self.assertIn("reliability_scheduler", replay)
        self.assertIn("play_command+=(/lio/diagnostics /lio/odom)", replay)
        self.assertIn('--clock "$REPLAY_CLOCK_HZ"', replay)
        self.assertIn("[lidar,gnss,imu,optical_flow,vision]", replay)
        self.assertIn("vision_health_provenance=recorded_missing_source_images", replay)
        self.assertIn('"native_worker_queue_discarded"', replay)

    def test_large_scene_rejects_low_rtf_before_flight(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")
        campaign = (
            REPO_ROOT / "tools" / "run_large_scene_validation.sh"
        ).read_text(encoding="utf-8")
        probe = (REPO_ROOT / "tools" / "topic_rate_probe.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("VALIDATION_MINIMUM_PREFLIGHT_RTF", runner)
        self.assertIn("VALIDATION_MINIMUM_PREFLIGHT_RTF", campaign)
        self.assertIn("--minimum-wall-source-ratio", runner)
        self.assertIn("minimum_wall_source_ratio", probe)
        self.assertIn('stop_collector "$resource_pid"', runner)
        self.assertIn("resource_metrics.json", runner)

    def test_validation_records_lio_dynamic_diagnostics(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("record_reliability_timeline.py", runner)
        self.assertIn('stop_collector "$timeline_pid"', runner)
        self.assertIn("/lio/diagnostics", runner)

    def test_validation_records_relocalization_point_cloud_inputs(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("/lio/local_map", runner)
        self.assertIn("/lidar/points_deskewed", runner)

    def test_truth_subscription_is_confined_to_route_controller(self):
        route = (
            SIM_ROOT
            / "multi_slam_uav_sim"
            / "guided_s_curve_waypoints.py"
        ).read_text(encoding="utf-8")
        self.assertIn('self.route_feedback_source == "gazebo_truth"', route)
        self.assertIn('"gazebo_truth_odom_topic"', route)

        runtime_roots = (
            ULTRA_ROOT / "uf_backend_fusion",
            ULTRA_ROOT / "uf_lio_adapter",
            ULTRA_ROOT / "uf_reliability",
            ULTRA_ROOT / "uf_visual_frontend",
        )
        leaking = []
        for root in runtime_roots:
            for path in root.rglob("*"):
                if not path.is_file() or "test" in path.parts:
                    continue
                if path.suffix not in {".py", ".cpp", ".hpp", ".yaml"}:
                    continue
                if TRUTH_TOPIC in path.read_text(
                    encoding="utf-8", errors="ignore"
                ):
                    leaking.append(str(path.relative_to(REPO_ROOT)))
        self.assertEqual(leaking, [])


if __name__ == "__main__":
    unittest.main()
