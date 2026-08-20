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

    def test_validation_separates_world_file_profile_from_gazebo_name(self):
        runner = (
            REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("VALIDATION_GAZEBO_WORLD_NAME", runner)
        self.assertIn('WORLD_NAME="$VALIDATION_GAZEBO_WORLD_NAME"', runner)
        self.assertIn('-p world_name:="$VALIDATION_GAZEBO_WORLD_NAME"', runner)

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
