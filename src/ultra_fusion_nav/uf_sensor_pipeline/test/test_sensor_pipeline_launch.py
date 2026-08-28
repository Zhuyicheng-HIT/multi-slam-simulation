import importlib.util
from pathlib import Path
import unittest

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


PATH = Path(__file__).resolve().parents[1] / "launch" / "sensor_pipeline.launch.py"
SPEC = importlib.util.spec_from_file_location("sensor_pipeline_launch", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SensorPipelineLaunchTest(unittest.TestCase):
    def test_production_manager_and_fault_switch_are_declared(self):
        description = MODULE.generate_launch_description()
        arguments = {
            action.name for action in description.entities
            if isinstance(action, DeclareLaunchArgument)
        }
        self.assertIn("enable_fault_injection", arguments)
        nodes = [
            action for action in description.entities if isinstance(action, Node)
        ]
        names = [action.__dict__.get("_Node__node_name") for action in nodes]
        self.assertIn("sensor_relay_manager", names)
        self.assertGreaterEqual(names.count("fault_injector_lidar"), 1)

    def test_manager_is_the_only_production_relay_entrypoint(self):
        source = PATH.read_text()
        self.assertIn('executable="sensor_relay_manager"', source)
        self.assertIn('condition=UnlessCondition(enable_fault_injection)', source)
        self.assertIn('condition=IfCondition(enable_fault_injection)', source)

    def test_minimal_profile_declares_all_manager_topics_before_start(self):
        description = MODULE.generate_launch_description()
        entities = list(description.entities)
        manager_index = next(
            index for index, action in enumerate(entities)
            if isinstance(action, Node)
            and action.__dict__.get("_Node__node_name") == "sensor_relay_manager"
        )
        declared_before_manager = {
            action.name for action in entities[:manager_index]
            if isinstance(action, DeclareLaunchArgument)
        }
        for argument in (
            "d435_depth_input_topic", "d435_color_input_topic",
            "active_modalities", "enable_vision", "enable_fault_injection",
        ):
            self.assertIn(argument, declared_before_manager)


if __name__ == "__main__":
    unittest.main()
