import ast
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


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

    def test_body_filter_keeps_reliable_input_compatibility_parameter(self):
        source = (PATH.parent.parent / "uf_sensor_pipeline" / "pointcloud_body_filter.py").read_text()
        self.assertIn('declare_parameter("reliable_input", False)', source)

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

    def test_every_launch_configuration_has_a_declaration(self):
        tree = ast.parse(PATH.read_text())
        configured = set()
        declared = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            function_name = getattr(node.func, "id", "")
            argument = node.args[0]
            if not isinstance(argument, ast.Constant) or not isinstance(argument.value, str):
                continue
            if function_name == "LaunchConfiguration":
                configured.add(argument.value)
            elif function_name == "DeclareLaunchArgument":
                declared.add(argument.value)
        self.assertEqual(configured - declared, set())


class MinimalLidarImuLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.previous_domain_id = os.environ.get("ROS_DOMAIN_ID")
        cls.previous_rmw = os.environ.get("RMW_IMPLEMENTATION")
        domain_id = 120 + os.getpid() % 20
        os.environ["ROS_DOMAIN_ID"] = str(domain_id)
        os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        rclpy.init(domain_id=domain_id)
        cls.node = rclpy.create_node("minimal_sensor_pipeline_launch_test")
        cls.received = []
        cls.subscription = cls.node.create_subscription(
            Imu,
            "/sensors/imu",
            cls.received.append,
            qos_profile_sensor_data,
        )
        cls.publisher = cls.node.create_publisher(
            Imu, "/livox/imu", qos_profile_sensor_data
        )
        cls.launch_log = tempfile.TemporaryFile(mode="w+")
        cls.launch_process = subprocess.Popen(
            [
                "ros2", "launch", "uf_sensor_pipeline", "sensor_pipeline.launch.py",
                "use_sim_time:=false",
                "enable_vision:=false",
                "enable_gnss:=false",
                "enable_lidar:=true",
                "enable_fault_injection:=false",
                "active_modalities:=[lidar,imu]",
                "imu_acceleration_scale:=1.0",
            ],
            stdout=cls.launch_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    @classmethod
    def tearDownClass(cls):
        if cls.launch_process.poll() is None:
            os.killpg(cls.launch_process.pid, signal.SIGINT)
            try:
                cls.launch_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(cls.launch_process.pid, signal.SIGKILL)
                cls.launch_process.wait(timeout=5.0)
        cls.launch_log.close()
        cls.node.destroy_node()
        rclpy.shutdown()
        if cls.previous_domain_id is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = cls.previous_domain_id
        if cls.previous_rmw is None:
            os.environ.pop("RMW_IMPLEMENTATION", None)
        else:
            os.environ["RMW_IMPLEMENTATION"] = cls.previous_rmw

    def test_minimal_profile_starts_and_relays_imu(self):
        deadline = time.monotonic() + 15.0
        manager_seen = False
        while time.monotonic() < deadline:
            if self.launch_process.poll() is not None:
                self.fail(f"sensor pipeline exited during startup:\n{self._launch_output()}")
            manager_seen = "sensor_relay_manager" in self.node.get_node_names()
            if (
                manager_seen
                and self.publisher.get_subscription_count() > 0
                and self.node.count_publishers("/sensors/imu") > 0
            ):
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(
            manager_seen,
            f"sensor_relay_manager did not start:\n{self._launch_output()}",
        )
        self.assertGreater(self.publisher.get_subscription_count(), 0)
        self.assertGreater(self.node.count_publishers("/sensors/imu"), 0)

        message = Imu()
        message.header.frame_id = "mid360_imu"
        message.linear_acceleration.x = 1.25
        message.linear_acceleration.y = -2.5
        message.linear_acceleration.z = 9.5
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not self.received:
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.received, "/livox/imu was not relayed to /sensors/imu")
        output = self.received[-1]
        self.assertEqual(output.header.frame_id, "mid360_imu")
        self.assertAlmostEqual(output.linear_acceleration.x, 1.25)
        self.assertAlmostEqual(output.linear_acceleration.y, -2.5)
        self.assertAlmostEqual(output.linear_acceleration.z, 9.5)
        node_names = self.node.get_node_names()
        self.assertIn("pointcloud_body_filter", node_names)
        self.assertFalse(any(name.startswith("fault_injector") for name in node_names))
        self.assertNotIn("d435i_mount_tf", node_names)
        self.assertNotIn("nmea_gnss", node_names)
        self.assertNotIn("gnss_metadata_relay", node_names)
        self.assertNotIn("fcu_observation_bridge", node_names)

    def _launch_output(self):
        self.launch_log.flush()
        self.launch_log.seek(0)
        return self.launch_log.read()


if __name__ == "__main__":
    unittest.main()
