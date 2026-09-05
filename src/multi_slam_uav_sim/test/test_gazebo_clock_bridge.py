import inspect
import threading
import unittest
from types import SimpleNamespace

from multi_slam_uav_sim.gazebo_clock_bridge import GazeboClockBridge


class GazeboClockBridgeConstructionTest(unittest.TestCase):
    def test_callback_state_is_initialized_before_subscription(self):
        source = inspect.getsource(GazeboClockBridge.__init__)

        state_index = source.index("self.last_sim_ns = None")
        subscribe_index = source.index("self.gz_node.subscribe(")

        self.assertLess(state_index, subscribe_index)

    def test_gazebo_callback_only_updates_thread_safe_cache(self):
        bridge = object.__new__(GazeboClockBridge)
        bridge._clock_lock = threading.Lock()
        bridge._latest_sim_stamp = None
        message = SimpleNamespace(sim=SimpleNamespace(sec=12, nsec=345))

        GazeboClockBridge._clock_cb(bridge, message)

        self.assertEqual(bridge._latest_sim_stamp, (12, 345))


if __name__ == "__main__":
    unittest.main()
