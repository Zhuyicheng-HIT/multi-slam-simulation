import math
import xml.etree.ElementTree as ET
from pathlib import Path

from multi_slam_uav_sim.s_curve_path import (
    generate_calibration_figure_eight,
    generate_large_figure_eight,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
WORLD = PACKAGE_ROOT / "worlds" / "simple_apm_rgbd_mid360.sdf"
LOW_WORLD = PACKAGE_ROOT / "worlds" / "low_indoor_apm_rgbd_mid360.sdf"
LANDMARKS = (
    PACKAGE_ROOT / "models" / "s_curve_lidar_landmarks" / "model.sdf"
)
URBAN_STRUCTURES = (
    PACKAGE_ROOT / "models" / "s_curve_urban_structures" / "model.sdf"
)
URBAN_TEXTURES = URBAN_STRUCTURES.parent / "materials" / "textures"
AIRCRAFT_MODEL = PACKAGE_ROOT / "models" / "iris_apm_rgbd" / "model.sdf"
D435I_DOWNWARD_MODEL = (
    PACKAGE_ROOT / "models" / "d435i_downward_rgbd" / "model.sdf"
)
S_CURVE_CONTROLLER = (
    PACKAGE_ROOT / "multi_slam_uav_sim" / "guided_s_curve_waypoints.py"
)
UNIFIED_FRONTEND_WRAPPER = REPO_ROOT / "tools" / "run_unified_fastlio_mapping.sh"
UNIFIED_BACKEND_WRAPPER = REPO_ROOT / "tools" / "run_unified_backend_stack.sh"
UNIFIED_VALIDATION = REPO_ROOT / "tools" / "run_unified_rectangle_validation.sh"
SIM_VISUAL_LAUNCH = (
    PACKAGE_ROOT / "launch" / "d435i_paper_visual_integration.launch.py"
)
SIM_VISUAL_RUNNER = (
    PACKAGE_ROOT / "scripts" / "run_pr6_d435i_visual_headless.sh"
)
FIGURE8_RUNNER = PACKAGE_ROOT / "scripts" / "run_s_curve_state_machine.sh"
VISUAL_FRONTEND_CONFIG = (
    REPO_ROOT / "src" / "ultra_fusion_nav" / "uf_visual_frontend"
    / "config" / "visual_frontend.yaml"
)
SHARED_MAPPING_CONFIG = (
    REPO_ROOT / "src" / "ultra_fusion_nav" / "uf_shared_mapping"
    / "config" / "shared_mapping.yaml"
)


def pose_xy(element):
    values = [float(value) for value in element.findtext("pose").split()]
    return values[0], values[1]


def pose_xyz(element):
    values = [float(value) for value in element.findtext("pose").split()]
    return values[0], values[1], values[2]


def named_children(root, tag, prefix):
    return [
        element for element in root.findall(f".//{tag}")
        if element.get("name", "").startswith(prefix)
    ]


def box_local_coordinates(point, collision):
    pose = [float(value) for value in collision.findtext("pose").split()]
    dx = point[0] - pose[0]
    dy = point[1] - pose[1]
    cosine = math.cos(pose[5])
    sine = math.sin(pose[5])
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        point[2] - pose[2],
    )


def box_clearance(point, collision):
    size = [
        float(value)
        for value in collision.findtext("geometry/box/size").split()
    ]
    local = box_local_coordinates(point, collision)
    outside = [
        max(abs(value) - extent * 0.5, 0.0)
        for value, extent in zip(local, size)
    ]
    return math.sqrt(sum(value * value for value in outside))


def cylinder_clearance(point, collision):
    pose = [float(value) for value in collision.findtext("pose").split()]
    radius = float(collision.findtext("geometry/cylinder/radius"))
    length = float(collision.findtext("geometry/cylinder/length"))
    radial_distance = math.hypot(point[0] - pose[0], point[1] - pose[1])
    radial_outside = max(radial_distance - radius, 0.0)
    vertical_outside = max(abs(point[2] - pose[2]) - length * 0.5, 0.0)
    return math.hypot(radial_outside, vertical_outside)


def collision_clearance(point, collision):
    if collision.find("geometry/box") is not None:
        return box_clearance(point, collision)
    if collision.find("geometry/cylinder") is not None:
        return cylinder_clearance(point, collision)
    raise AssertionError(f"unsupported collision geometry: {collision.get('name')}")


def path_heading(path, index):
    lower = max(0, index - 1)
    upper = min(len(path) - 1, index + 1)
    return math.atan2(
        path[upper][1] - path[lower][1],
        path[upper][0] - path[lower][0],
    )


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
    assert "optical_flow_imu" not in sensors
    assert "front_d435i_imu" not in sensors
    for name in ("optical_flow_mono_down", "optical_flow_range"):
        pose = [float(value) for value in sensors[name].findtext("pose").split()]
        assert len(pose) == 6
        assert math.isclose(pose[3], 0.0, abs_tol=1.0e-9)
        assert math.isclose(pose[4], math.pi / 2.0, abs_tol=1.0e-9)
        assert math.isclose(pose[5], 0.0, abs_tol=1.0e-9)

        # Gazebo sensors look along local +X. R_y(+pi/2) maps it to body -Z.
        forward_body_z = -math.sin(pose[4])
        assert forward_body_z < -0.999999


def test_mid360_mount_is_fifteen_degrees_nose_down():
    root = ET.parse(AIRCRAFT_MODEL).getroot()
    link = root.find(".//link[@name='mid360_link']")
    assert link is not None
    pose = [float(value) for value in link.findtext("pose").split()]
    assert len(pose) == 6
    assert math.isclose(pose[0], 0.05, abs_tol=1.0e-9)
    assert math.isclose(pose[2], 0.10, abs_tol=1.0e-9)
    assert math.isclose(pose[4], math.radians(15.0), abs_tol=1.0e-9)


def test_rgbd_cameras_run_at_fifteen_hz_without_throttling_optical_flow():
    aircraft = ET.parse(AIRCRAFT_MODEL).getroot()
    sensors = {
        sensor.get("name"): sensor
        for sensor in aircraft.findall(".//sensor")
    }
    assert float(sensors["front_d435i_rgbd"].findtext("update_rate")) == 15.0
    assert float(sensors["optical_flow_mono_down"].findtext("update_rate")) == 30.0
    assert float(sensors["optical_flow_range"].findtext("update_rate")) == 30.0

    downward = ET.parse(D435I_DOWNWARD_MODEL).getroot()
    rgbd = downward.find(".//sensor[@name='d435i_rgbd_down']")
    assert rgbd is not None
    assert float(rgbd.findtext("update_rate")) == 15.0


def test_low_world_has_nonplanar_ground_for_range_facets():
    root = ET.parse(LOW_WORLD).getroot()
    collisions = root.findall(".//collision")
    reliefs = [
        collision for collision in collisions
        if collision.get("name", "").startswith("relief_")
    ]
    assert len(reliefs) >= 8
    heights = []
    for collision in reliefs:
        geometry = collision.find("geometry")
        assert geometry is not None
        if geometry.find("box") is not None:
            heights.append(float(geometry.findtext("box/size").split()[2]))
        elif geometry.find("cylinder") is not None:
            heights.append(float(geometry.findtext("cylinder/length")))
        else:
            raise AssertionError("unsupported relief geometry")
    assert max(heights) >= 0.20
    assert max(heights) <= 0.50


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


def test_lidar_landmarks_cover_the_large_figure_eight_and_outer_area():
    root = ET.parse(LANDMARKS).getroot()
    landmarks = root.findall(".//collision")
    assert not root.findall(".//visual"), (
        "LiDAR observability landmarks must not occlude the persisted RTAB "
        "camera scene"
    )
    positions = [pose_xy(item) for item in landmarks]
    assert len(positions) >= 18
    assert max(max(abs(value) for value in point) for point in positions) >= 15.0

    route = generate_large_figure_eight(
        9.0, 1.5, 5.0, 4.5, samples=481,
        rotation_deg=158.0, altitude_power=4)
    route_max_distance = max(
        min(math.dist((x, y), landmark) for landmark in positions)
        for x, y, _ in route
    )
    assert route_max_distance <= 4.5


def test_low_figure_eight_stays_below_five_metres():
    route = generate_large_figure_eight(
        9.0, 1.5, 2.2, 0.8, samples=481,
        rotation_deg=158.0, altitude_power=4)
    altitudes = [point[2] for point in route]
    assert min(altitudes) >= 2.2 - 1.0e-9
    assert max(altitudes) <= 3.0 + 1.0e-9
    assert sum(value <= 5.0 for value in altitudes) / len(altitudes) >= 0.50

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
        "east_canyon_gateway_south_pier",
        "east_canyon_gateway_north_pier",
        "east_canyon_gateway_header",
        "north_arcade_header",
        "south_visual_totem",
        "north_service_block",
        "east_background_facade",
        "west_background_facade",
        "north_background_facade",
        "south_background_facade",
        "east_wall_relief_01",
        "west_wall_relief_03",
        "north_wall_relief_05",
        "south_wall_relief_01",
    }
    assert required <= collision_names
    assert len(collision_names) >= 36

    albedo_maps = {
        element.text
        for element in urban_root.findall(".//albedo_map")
    }
    expected_textures = {
        "materials/textures/facade_a_v2.png",
        "materials/textures/facade_b_v2.png",
        "materials/textures/tunnel_v1.png",
        "materials/textures/canyon_v1.png",
    }
    assert expected_textures <= albedo_maps
    for relative_path in expected_textures:
        texture = URBAN_STRUCTURES.parent / relative_path
        assert texture.is_file()
        assert texture.stat().st_size > 100_000

    textured_visuals = {
        visual.get("name"): visual.findtext("material/pbr/metal/albedo_map")
        for visual in urban_root.findall(".//visual")
    }
    assert textured_visuals["short_tunnel_left_wall_visual"] == (
        "materials/textures/tunnel_v1.png"
    )
    assert textured_visuals["urban_canyon_east_south_visual"] == (
        "materials/textures/canyon_v1.png"
    )
    assert textured_visuals["south_east_facade_visual"] == (
        "materials/textures/facade_a_v2.png"
    )


def test_forward_visual_geometry_covers_the_figure_eight_at_simulation_range():
    urban_root = ET.parse(URBAN_STRUCTURES).getroot()
    features = named_children(urban_root, "visual", "visual_feature_")
    assert len(features) >= 40

    anchors = [pose_xyz(feature) for feature in features]
    route = generate_large_figure_eight(
        9.0, 1.5, 5.0, 4.5, samples=481,
        rotation_deg=158.0, altitude_power=4)
    half_horizontal_fov = math.radians(35.0)
    half_vertical_fov = math.radians(28.0)
    midpoint = len(route) // 2
    locked_second_lobe_yaw = path_heading(route, midpoint)

    maximum_visible_distance = 0.0
    for index, point in enumerate(route):
        yaw = (
            path_heading(route, index)
            if index <= midpoint
            else locked_second_lobe_yaw
        )
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        visible = []
        for anchor in anchors:
            dx = anchor[0] - point[0]
            dy = anchor[1] - point[1]
            dz = anchor[2] - point[2]
            forward = cosine * dx + sine * dy
            lateral = -sine * dx + cosine * dy
            if forward <= 0.20:
                continue
            if abs(lateral) > forward * math.tan(half_horizontal_fov) + 0.80:
                continue
            horizontal_distance = math.hypot(dx, dy)
            if abs(dz) > (
                horizontal_distance * math.tan(half_vertical_fov) + 0.50
            ):
                continue
            distance = math.dist(point, anchor)
            if distance <= 10.0:
                visible.append(distance)
        assert visible, f"no forward visual geometry near route point {point}"
        maximum_visible_distance = max(maximum_visible_distance, min(visible))

    assert maximum_visible_distance <= 10.0


def test_simulation_depth_range_is_extended_without_changing_base_profiles():
    launch = SIM_VISUAL_LAUNCH.read_text(encoding="utf-8")
    runner = SIM_VISUAL_RUNNER.read_text(encoding="utf-8")
    visual_config = VISUAL_FRONTEND_CONFIG.read_text(encoding="utf-8")
    mapping_config = SHARED_MAPPING_CONFIG.read_text(encoding="utf-8")

    assert '"rgbd_maximum_depth_m", default_value="10.0"' in launch
    assert "SIM_RGBD_MAX_DEPTH_M=${SIM_RGBD_MAX_DEPTH_M:-10.0}" in runner
    assert "maximum_depth_m: 6.0" in visual_config
    assert "maximum_depth_m: 6.0" in mapping_config


def test_three_dimensional_route_has_physical_clearance_from_urban_boxes():
    urban_root = ET.parse(URBAN_STRUCTURES).getroot()
    boxes = urban_root.findall(".//collision")
    route = generate_large_figure_eight(
        9.0, 1.5, 5.0, 4.5, samples=961,
        rotation_deg=158.0, altitude_power=4)
    minimum_clearance = min(
        box_clearance(point, collision)
        for point in route
        for collision in boxes
    )
    # This is centerline clearance. The route controller additionally limits
    # command offsets and flies at low speed; the model opening remains wide
    # enough for the Iris body and companion-sensor envelope.
    assert minimum_clearance >= 0.75


def test_new_urban_structures_keep_the_figure_eight_corridor_open():
    urban_root = ET.parse(URBAN_STRUCTURES).getroot()
    added_names = {
        "north_arcade_header",
        "south_visual_totem",
        "north_service_block",
        "east_canyon_gateway_south_pier",
        "east_canyon_gateway_north_pier",
        "east_canyon_gateway_header",
        "east_background_facade",
        "west_background_facade",
        "north_background_facade",
        "south_background_facade",
    }
    added = [
        collision for collision in urban_root.findall(".//collision")
        if collision.get("name") in added_names
    ]
    assert {collision.get("name") for collision in added} == added_names

    route = generate_large_figure_eight(
        9.0, 1.5, 5.0, 4.5, samples=961,
        rotation_deg=158.0, altitude_power=4)
    minimum_clearance = min(
        box_clearance(point, collision)
        for point in route
        for collision in added
    )
    assert minimum_clearance >= 0.75


def test_second_lobe_straight_exits_through_the_gateway_opening():
    urban_root = ET.parse(URBAN_STRUCTURES).getroot()
    header = next(
        collision for collision in urban_root.findall(".//collision")
        if collision.get("name") == "east_canyon_gateway_header"
    )
    route = generate_large_figure_eight(
        9.0, 1.5, 5.0, 4.5, samples=2001,
        rotation_deg=158.0, altitude_power=4)
    crossings = []
    for index, (first, second) in enumerate(zip(route[:-1], route[1:])):
        first_x = box_local_coordinates(first, header)[0]
        second_x = box_local_coordinates(second, header)[0]
        if first_x * second_x > 0.0 or math.isclose(first_x, second_x):
            continue
        ratio = first_x / (first_x - second_x)
        point = tuple(
            first[axis] + ratio * (second[axis] - first[axis])
            for axis in range(3)
        )
        local = box_local_coordinates(point, header)
        progress = (index + ratio) / (len(route) - 1)
        if abs(local[1]) <= 1.10 and 0.35 <= point[2] <= 6.60:
            crossings.append((progress, point, local))

    assert len(crossings) == 1
    progress, point, local = crossings[0]
    assert 0.50 < progress < 0.75
    assert abs(local[1]) <= 0.05
    assert math.isclose(point[2], 5.0, abs_tol=1.0e-6)


def test_visual_diagnostic_rectangle_keeps_the_collision_audited_size():
    runner = SIM_VISUAL_RUNNER.read_text(encoding="utf-8")
    assert "RECTANGLE_LENGTH_X=${RECTANGLE_LENGTH_X:-2.0}" in runner
    assert "RECTANGLE_LENGTH_Y=${RECTANGLE_LENGTH_Y:-1.2}" in runner


def test_figure_eight_runner_keeps_single_pass_geometry_and_yaw_contract():
    runner = FIGURE8_RUNNER.read_text(encoding="utf-8")
    controller = S_CURVE_CONTROLLER.read_text(encoding="utf-8")
    assert "RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp" in runner
    assert "S_CURVE_SPAN=${S_CURVE_SPAN:-9.0}" in runner
    assert "S_CURVE_AMPLITUDE=${S_CURVE_AMPLITUDE:-1.5}" in runner
    assert "S_CURVE_VERTICAL_AMPLITUDE=${S_CURVE_VERTICAL_AMPLITUDE:-0.8}" in runner
    assert "S_CURVE_PASSES=${S_CURVE_PASSES:-1}" in runner
    assert "FIGURE8_ROTATION_DEG=${FIGURE8_ROTATION_DEG:-158.0}" in runner
    assert "follow_heading_fraction=0.5" in controller
    assert "yaw_mode=first_lobe_heading_follow/second_lobe_locked" in controller


def test_full_mission_flies_one_closed_figure_eight_without_retracing():
    urban_root = ET.parse(URBAN_STRUCTURES).getroot()
    boxes = urban_root.findall(".//collision")
    route = generate_large_figure_eight(
        9.0, 1.5, 5.0, 4.5, samples=961,
        rotation_deg=158.0, altitude_power=4)
    home = (0.0, 0.0, 5.0)
    assert route[0] == home
    assert route[len(route) // 2] == home
    assert route[-1] == home
    takeoff = sample_segment((0.0, 0.0, 0.25), home)
    calibration = generate_calibration_figure_eight(home, 1.0, samples=161)
    mission = sample_polyline(
        takeoff + calibration[1:] + route[1:]
    )
    minimum_clearance = min(
        box_clearance(point, collision)
        for point in mission
        for collision in boxes
    )
    assert minimum_clearance >= 0.75

    controller = S_CURVE_CONTROLLER.read_text(encoding="utf-8")
    assert "large figure-eight single traversal" in controller
    assert "list(reversed(base_path))" not in controller


def test_vertical_diagnostic_uses_a_clear_corridor_outside_the_tunnel_roof():
    urban_root = ET.parse(URBAN_STRUCTURES).getroot()
    landmark_root = ET.parse(LANDMARKS).getroot()
    obstacles = (
        urban_root.findall(".//collision")
        + landmark_root.findall(".//collision")
    )
    home = (0.0, 0.0, 5.0)
    staging = (-4.3, 0.9, 5.0)
    peak = (-4.3, 0.9, 9.5)
    route = sample_polyline([home, staging, peak, staging, home])

    minimum_clearance = min(
        collision_clearance(point, collision)
        for point in route
        for collision in obstacles
    )

    # The audited centerline stays at least 1.32 m from every modeled obstacle.
    # A conservative 0.50 m vehicle sphere therefore retains over 0.82 m.
    assert minimum_clearance >= 1.30


def test_s_curve_navigation_feedback_is_strictly_the_unified_backend():
    source = S_CURVE_CONTROLLER.read_text(encoding="utf-8")
    assert '"unified_odom_topic", "/fusion/unified/odom"' in source
    assert '"route_feedback_source", "unified_backend"' in source
    assert '"gazebo_truth_odom_topic", "/sim/mid360/ground_truth_odom"' in source
    assert 'self.route_feedback_source == "gazebo_truth"' in source
    assert 'self.route_feedback_source == "unified_backend"' in source
    assert "effective_hold = mission_hold_required(" in source
    assert "decision.hold, lost, self.relocalization_request_active" in source
    assert "route_hold_fcu_setpoint" in source
    assert "route_altitude_margin_m" in source


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
    assert "VALIDATION_RELIABILITY_MODE:-dynamic" in validation
    assert 'RELIABILITY_MODE="$VALIDATION_RELIABILITY_MODE"' in validation
