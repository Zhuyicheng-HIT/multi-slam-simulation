from types import SimpleNamespace

from pymavlink import mavutil

from multi_slam_uav_sim.guided_rectangle_waypoints import (
    GuidedRectangleWaypoints,
    ekf_flags_have_absolute_position,
)


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
