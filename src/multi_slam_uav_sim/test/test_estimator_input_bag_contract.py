import importlib.util
from pathlib import Path
import unittest


TOOL_PATH = Path(__file__).resolve().parents[3] / "tools" / "verify_estimator_input_bag.py"
CAPTURE_TOOL_PATH = (
    Path(__file__).resolve().parents[3]
    / "tools"
    / "capture_full_online_backend_replay.sh"
)
SPEC = importlib.util.spec_from_file_location("verify_estimator_input_bag", TOOL_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def metadata(*topics):
    return {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": topic},
                    "message_count": 1,
                }
                for topic in topics
            ]
        }
    }


class EstimatorInputBagContractTest(unittest.TestCase):
    def test_full_capture_records_rgbd_direct_input(self):
        capture_script = CAPTURE_TOOL_PATH.read_text(encoding="utf-8")
        self.assertIn("/vision/rgbd_direct_tracks", capture_script)

    def test_rgbd_geometry_is_optional_for_four_source_replay(self):
        report = MODULE.build_report(metadata(*MODULE.CORE_TOPICS))
        self.assertTrue(report["valid"])
        self.assertEqual(report["rgbd_geometry_count"], 0)

    def test_rgbd_geometry_is_required_for_metric_depth_replay(self):
        report = MODULE.build_report(
            metadata(*MODULE.CORE_TOPICS, *MODULE.VISUAL_TOPICS),
            require_visual=True,
            require_rgbd_geometry=True,
        )
        self.assertFalse(report["valid"])
        self.assertEqual(
            report["missing_or_empty_topics"],
            [MODULE.RGBD_GEOMETRY_TOPIC],
        )

    def test_nonempty_rgbd_geometry_satisfies_metric_depth_contract(self):
        report = MODULE.build_report(
            metadata(
                *MODULE.CORE_TOPICS,
                *MODULE.VISUAL_TOPICS,
                MODULE.RGBD_GEOMETRY_TOPIC,
            ),
            require_visual=True,
            require_rgbd_geometry=True,
        )
        self.assertTrue(report["valid"])
        self.assertEqual(report["rgbd_geometry_count"], 1)


if __name__ == "__main__":
    unittest.main()
