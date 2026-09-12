import importlib.util
from pathlib import Path
import unittest

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "online_backend.launch.py"
)
SPEC = importlib.util.spec_from_file_location("online_backend_launch", LAUNCH_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class OnlineBackendLaunchTest(unittest.TestCase):
    def test_output_mode_argument_has_fixed_default(self):
        description = MODULE.generate_launch_description()
        arguments = {
            action.name: action.default_value
            for action in description.entities
            if isinstance(action, DeclareLaunchArgument)
        }
        self.assertEqual(
            arguments["unified_odom_output_mode"][0].perform(None),
            "fixed_rate_propagated",
        )

    def test_output_mode_is_bound_to_backend_node(self):
        description = MODULE.generate_launch_description()
        backend = next(
            action
            for action in description.entities
            if isinstance(action, Node)
            and action.__dict__.get("_Node__node_name") == "unified_backend_fusion"
        )
        parameter_maps = [parameter for parameter in backend.__dict__.get(
            "_Node__parameters", []
        ) if isinstance(parameter, dict)]
        mode = next(
            value
            for parameter in parameter_maps
            for key, value in parameter.items()
            if "unified_odom_output_mode" in str(
                key[0].__dict__.get("_TextSubstitution__text", "")
            )
        )
        if isinstance(mode, tuple):
            mode = mode[0]
        self.assertIsInstance(mode, LaunchConfiguration)
        self.assertEqual(
            mode.__dict__["_LaunchConfiguration__variable_name"][0].perform(None),
            "unified_odom_output_mode",
        )

    def test_supported_modes_are_documented(self):
        source = LAUNCH_PATH.read_text()
        self.assertIn("fixed_rate_propagated", source)
        self.assertIn("LiDAR-event-triggered", source)


if __name__ == "__main__":
    unittest.main()
