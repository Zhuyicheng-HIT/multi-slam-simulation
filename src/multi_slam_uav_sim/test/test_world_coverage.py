import math
import xml.etree.ElementTree as ET
from pathlib import Path

from multi_slam_uav_sim.s_curve_path import (
    generate_calibration_figure_eight,
    generate_s_curve,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
WORLD = PACKAGE_ROOT / "worlds" / "simple_apm_rgbd_mid360.sdf"
LANDMARKS = (
    PACKAGE_ROOT / "models" / "s_curve_lidar_landmarks" / "model.sdf"
)
URBAN_STRUCTURES = (
    PACKAGE_ROOT / "models" / "s_curve_urban_structures" / "model.sdf"
)
AIRCRAFT_MODEL = PACKAGE_ROOT / "models" / "iris_apm_rgbd" / "model.sdf"
S_CURVE_CONTROLLER = (
    PACKAGE_ROOT / "multi_slam_uav_sim" / "guided_s_curve_waypoints.py"
)
UNIFIED_FRONTEND_WRAPPER = REPO_ROOT / "tools" / "run_unified_fastlio_mapping.sh"
UNIFIED_BACKEND_WRAPPER = REPO_ROOT / "tools" / "run_unified_backend_stack.sh"
UNIFIED_VALIDATION = REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"


def pose_xy(element):
    values = [float(value) for value in element.findtext("pose").split()]
    return values[0], values[1]


def named_children(root, tag, prefix):
    return [
        element for element in root.findall(f".//{tag}")
        if element.get("name", "").startswith(prefix)
    ]


def box_clearance(point, collision):
    pose = [float(value) for value in collision.findtext("pose").split()]
    size = [
        float(value)
        for value in collision.findtext("geometry/box/size").split()
    ]
    dx = point[0] - pose[0]
    dy = point[1] - pose[1]
    cosine = math.cos(pose[5])
    sine = math.sin(pose[5])
    local = (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        point[2] - pose[2],
    )
    outside = [
        max(abs(value) - extent * 0.5, 0.0)
        for value, extent in zip(local, size)
    ]
    return math.sqrt(sum(value * value for value in outside))


def sample_segment(start, end, spacing=0.05):
    count = max(2, math.ceil(math.dist(start, end) / spacing) + 1)
    return [
        tuple(
            start[axis] + (end[axis] - start[axis]) * index / (count - 1)
            for axis in range(3)
        )
        for index in range(count)
    ]


def sample_polyline(points, spacing=0.05):
    sampled = [points[0]]
    for point in points[1:]:
        sampled.extend(sample_segment(sampled[-1], point, spacing)[1:])
    return sampled


def test_optical_flow_camera_and_range_point_downward():
    root = ET.parse(AIRCRAFT_MODEL).getroot()
    sensors = {
        sensor.get("name"): sensor
        for sensor in root.findall(".//sensor")
    }
    for name in ("optical_flow_mono_down", "optical_flow_range"):
        pose = [float(value) for value in sensors[name].findtext("pose").split()]
        assert len(pose) == 6
        assert math.isclose(pose[3], 0.0, abs_tol=1.0e-9)
        assert math.isclose(pose[4], math.pi / 2.0, abs_tol=1.0e-9)
        assert math.isclose(pose[5], 0.0, abs_tol=1.0e-9)

        # Gazebo sensors look along local +X. R_y(+pi/2) maps it to body -Z.
        forward_body_z = -math.sin(pose[4])
        assert forward_body_z < -0.999999


def test_companion_sensor_payloads_are_dynamically_negligible():
    root = ET.parse(AIRCRAFT_MODEL).getroot()
    links = {
        link.get("name"): link
        for link in root.findall(".//link")
    }

    for name in ("flow_camera_link", "front_d435i_link", "mid360_link"):
        inertial = links[name].find("inertial")
        assert inertial is not None

        # Gazebo requires positive inertial values for a dynamic link. These
        # fixed payload links therefore use near-zero values instead of zero.
        mass = float(inertial.findtext("mass"))
        assert 0.0 < mass <= 1.0e-6
        for axis in ("ixx", "iyy", "izz"):
            moment = float(inertial.findtext(f"inertia/{axis}"))
            assert 0.0 < moment <= 1.0e-9


def test_flow_texture_covers_the_expanded_floor():
    root = ET.parse(WORLD).getroot()
    x_lines = named_children(root, "visual", "x_grid_")
    y_lines = named_children(root, "visual", "y_grid_")
    assert len(x_lines) == 19
    assert len(y_lines) == 19
    assert min(pose_xy(item)[0] for item in x_lines) <= -18.0
    assert max(pose_xy(item)[0] for item in x_lines) >= 18.0
    assert min(pose_xy(item)[1] for item in y_lines) <= -18.0
    assert max(pose_xy(item)[1] for item in y_lines) >= 18.0
    outer_lines = [
        item for item in x_lines + y_lines
        if max(abs(value) for value in pose_xy(item)) > 8.0
    ]
    assert all(
        max(float(value) for value in item.findtext("geometry/box/size").split())
        >= 38.0
        for item in outer_lines
    )

    flow_markers = named_children(root, "visual", "marker_")
    assert len(flow_markers) >= 32
    assert max(max(abs(value) for value in pose_xy(item)) for item in flow_markers) >= 15.0


def test_central_visual_grid_keeps_the_persisted_rtab_contract():
    root = ET.parse(WORLD).getroot()
    visuals = {
        element.get("name"): element
        for element in root.findall(".//visual")
    }
    for axis in ("x", "y"):
        for coordinate in range(-8, 9, 2):
            element = visuals[f"{axis}_grid_{coordinate}"]
            size = [
                float(value)
                for value in element.findtext("geometry/box/size").split()
            ]
            expected_length = 16.0 if axis == "x" else 18.0
            assert math.isclose(max(size), expected_length, abs_tol=1.0e-9)
            assert element.find("material/ambient") is not None


def test_lidar_landmarks_cover_the_s_curve_and_outer_area():
    root = ET.parse(LANDMARKS).getroot()
    landmarks = root.findall(".//collision")
    assert not root.findall(".//visual"), (
        "LiDAR observability landmarks must not occlude the persisted RTAB "
        "camera scene"
    )
    positions = [pose_xy(item) for item in landmarks]
    assert len(positions) >= 18
    assert max(max(abs(value) for value in point) for point in positions) >= 15.0

    route = generate_s_curve(12.0, 4.5, 5.0, 1.0, samples=241)
    route_max_distance = max(
        min(math.dist((x, y), landmark) for landmark in positions)
        for x, y, _ in route
    )
    assert route_max_distance <= 4.5

    audit_grid = [
        (x, y)
        for x in (-16.0, -8.0, 0.0, 8.0, 16.0)
        for y in (-16.0, -8.0, 0.0, 8.0, 16.0)
    ]
    expanded_max_distance = max(
        min(math.dist(point, landmark) for landmark in positions)
        for point in audit_grid
    )
    assert expanded_max_distance <= 8.5


def test_urban_structures_are_loaded_and_replace_the_pillar_forest():
    world_root = ET.parse(WORLD).getroot()
    includes = {
        include.findtext("uri") for include in world_root.findall(".//include")
    }
    assert "model://s_curve_urban_structures" in includes

    landmark_root = ET.parse(LANDMARKS).getroot()
    assert not named_children(landmark_root, "collision", "route_marker_")

    urban_root = ET.parse(URBAN_STRUCTURES).getroot()
    collision_names = {
        collision.get("name")
        for collision in urban_root.findall(".//collision")
    }
    required = {
        "south_arch_left_pier",
        "south_arch_right_pier",
        "south_arch_header",
        "short_tunnel_left_wall",
        "short_tunnel_right_wall",
        "short_tunnel_roof",
        "urban_canyon_west_south",
        "urban_canyon_east_south",
    }
    assert required <= collision_names
    assert len(collision_names) >= 14


def test_three_dimensional_route_has_physical_clearance_from_urban_boxes():
    urban_root = ET.parse(URBAN_STRUCTURES).getroot()
    boxes = urban_root.findall(".//collision")
    route = generate_s_curve(
        12.0, 4.5, 5.0, 1.0, samples=481, vertical_cycles=2)
    minimum_clearance = min(
        box_clearance(point, collision)
        for point in route
        for collision in boxes
    )
    # This is centerline clearance. The route controller additionally limits
    # command offsets and flies at low speed; the model opening remains wide
    # enough for the Iris body and companion-sensor envelope.
    assert minimum_clearance >= 1.00


def test_full_mission_uses_the_s_curve_for_collision_free_entry_and_return():
    urban_root = ET.parse(URBAN_STRUCTURES).getroot()
    boxes = urban_root.findall(".//collision")
    route = generate_s_curve(
        12.0, 4.5, 5.0, 1.0, samples=481, vertical_cycles=2)
    home = (0.0, 0.0, 5.0)
    anchor_index = min(
        range(len(route)), key=lambda index: math.dist(route[index], home))
    assert math.dist(route[anchor_index], home) <= 1.0e-9

    entry = list(reversed(route[:anchor_index + 1]))
    return_to_home = list(reversed(route[anchor_index:]))
    takeoff = sample_segment((0.0, 0.0, 0.25), home)
    calibration = generate_calibration_figure_eight(home, 1.0, samples=161)
    mission = sample_polyline(
        takeoff + calibration[1:] + entry[1:]
        + route[1:] + list(reversed(route))[1:] + route[1:]
        + return_to_home[1:]
    )
    minimum_clearance = min(
        box_clearance(point, collision)
        for point in mission
        for collision in boxes
    )
    assert minimum_clearance >= 1.00

    # A direct home-to-endpoint shortcut intersects a tunnel side wall. This
    # guards against restoring the old straight transit/return behavior.
    direct_shortcut = sample_segment(home, route[0])
    assert min(
        box_clearance(point, collision)
        for point in direct_shortcut
        for collision in boxes
    ) == 0.0


def test_s_curve_navigation_feedback_is_strictly_the_unified_backend():
    source = S_CURVE_CONTROLLER.read_text(encoding="utf-8")
    assert '"unified_odom_topic", "/fusion/unified/odom"' in source
    assert "route_feedback_source=unified_backend" in source
    assert "ground_truth" not in source
    assert "/sim/" not in source
    assert "effective_hold = decision.hold or lost" in source
    assert "route_hold_fcu_setpoint" in source


def test_stable_unified_launch_exports_native_factors_without_scan_handshake():
    frontend = UNIFIED_FRONTEND_WRAPPER.read_text(encoding="utf-8")
    backend = UNIFIED_BACKEND_WRAPPER.read_text(encoding="utf-8")
    validation = UNIFIED_VALIDATION.read_text(encoding="utf-8")

    assert "FASTLIO_NATIVE_FACTOR_EXPORT:-1" in frontend
    assert "FASTLIO_DOWNSTREAM_BACKEND:-1" in frontend
    assert "FASTLIO_MAP_INSERTION_MODE:-backend_confirmed" in frontend
    assert "FASTLIO_BACKEND_TRAJECTORY_FRONTEND:-0" in frontend
    assert "FRONTEND_SCAN_PREDICTION_ENABLED:-false" in backend
    assert "FASTLIO_BACKEND_TRAJECTORY_FRONTEND:-0" in validation
