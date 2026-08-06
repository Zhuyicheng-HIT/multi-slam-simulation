from types import SimpleNamespace

from pymavlink import mavutil

from multi_slam_uav_sim.guided_rectangle_waypoints import (
    GuidedRectangleWaypoints,
    ekf_flags_have_absolute_position,
)
import multi_slam_uav_sim.guided_rectangle_waypoints as waypoint_module


def _node(*, fix_status, ekf_using_gps, navigation_source="gps"):
    node = GuidedRectangleWaypoints.__new__(GuidedRectangleWaypoints)
    node.fix = (
        None
        if fix_status is None
        else SimpleNamespace(status=SimpleNamespace(status=fix_status))
    )
    node.ekf_using_gps = ekf_using_gps
    node.ekf_absolute_position_ready = False
    node.navigation_source = navigation_source
    node.last_flow_time = None
    return node


def test_gps_fix_alone_does_not_release_navigation_gate():
    node = _node(fix_status=0, ekf_using_gps=False)

    assert node._gps_ready()
    assert not node._gps_navigation_ready()
    assert node._navigation_source() is None


def test_ekf_gps_acceptance_releases_navigation_gate():
    node = _node(fix_status=0, ekf_using_gps=True)

    assert node._gps_navigation_ready()
    assert node._navigation_source() == "gps"


def test_invalid_fix_stays_blocked_after_old_ekf_status_text():
    node = _node(fix_status=-1, ekf_using_gps=True)

    assert not node._gps_navigation_ready()
    assert node._navigation_source() is None


def test_current_ekf_absolute_position_releases_gate_after_status_text_was_missed():
    node = _node(fix_status=0, ekf_using_gps=False)
    node.ekf_absolute_position_ready = True

    assert node._gps_navigation_ready()
    assert node._navigation_source() == "gps"


def test_navigation_wait_survives_initial_ros_clock_jump(monkeypatch):
    node = GuidedRectangleWaypoints.__new__(GuidedRectangleWaypoints)
    state = {"wall": 0.0, "ros": 0.0, "spins": 0}
    node.preflight_wait_s = 45.0
    node.navigation_stable_s = 1.0
    node.navigation_source = "gps"
    node.flow_min_quality = 0
    node.ekf_using_gps = True
    node.ekf_absolute_position_ready = True
    node.fix = SimpleNamespace(status=SimpleNamespace(status=0))
    node.pose = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
        )
    )
    node._now_s = lambda: state["ros"]
    node._log_status = lambda _prefix: None

    class Logger:
        def info(self, _message):
            pass

    node.get_logger = lambda: Logger()

    def spin_once(_node, timeout_sec):
        assert timeout_sec == 0.1
        state["spins"] += 1
        state["wall"] += 0.1
        state["ros"] = 90.0 if state["spins"] == 1 else 91.1

    monkeypatch.setattr(waypoint_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(waypoint_module.rclpy, "spin_once", spin_once)
    monkeypatch.setattr(
        waypoint_module.time, "monotonic", lambda: state["wall"]
    )

    assert node.wait_navigation_ready() == "gps"
    assert state["spins"] == 2
    assert (node.home_x, node.home_y, node.home_z) == (1.0, 2.0, 3.0)


def test_ekf_flags_reject_glitch_and_uninitialized_states():
    healthy = (
        mavutil.mavlink.EKF_ATTITUDE
        | mavutil.mavlink.EKF_VELOCITY_HORIZ
        | mavutil.mavlink.EKF_POS_HORIZ_ABS
    )

    assert ekf_flags_have_absolute_position(healthy)
    assert not ekf_flags_have_absolute_position(
        healthy | mavutil.mavlink.EKF_GPS_GLITCHING
    )
    assert not ekf_flags_have_absolute_position(
        healthy | mavutil.mavlink.EKF_UNINITIALIZED
    )


def test_command_phase_retries_timed_out_mode_and_arm_services(monkeypatch):
    node = GuidedRectangleWaypoints.__new__(GuidedRectangleWaypoints)
    node.command_retry_s = 60.0
    node.state = SimpleNamespace(mode="STABILIZE", armed=False, connected=True)
    node.mode_cli = object()
    node.arming_cli = object()
    node.home_x = 0.0
    node.home_y = 0.0
    node.home_yaw = 0.0
    node.takeoff_alt = 3.0
    calls = {"mode": 0, "arm": 0}

    class Logger:
        def info(self, _message):
            pass

        def warning(self, _message):
            pass

    node.get_logger = lambda: Logger()
    node.hold_setpoint = lambda *_args, **_kwargs: None
    node.wait_for_state = lambda predicate, *_args, **_kwargs: predicate()
    node.send_takeoff_command_int = lambda: True

    def call(client, _request, _label):
        if client is node.mode_cli:
            calls["mode"] += 1
            if calls["mode"] == 1:
                raise RuntimeError("Timeout calling set GUIDED")
            node.state.mode = "GUIDED"
            return SimpleNamespace(mode_sent=True)
        calls["arm"] += 1
        if calls["arm"] == 1:
            raise RuntimeError("Timeout calling arm")
        node.state.armed = True
        return SimpleNamespace(success=True)

    node.call = call
    monkeypatch.setattr(waypoint_module.rclpy, "ok", lambda: True)

    node.set_guided_arm_takeoff()

    assert calls == {"mode": 2, "arm": 2}
