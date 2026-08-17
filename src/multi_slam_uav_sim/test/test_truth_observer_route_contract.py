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
