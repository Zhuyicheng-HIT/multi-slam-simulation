import math
import xml.etree.ElementTree as ET
from pathlib import Path

from multi_slam_uav_sim.s_curve_path import generate_s_curve


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD = PACKAGE_ROOT / "worlds" / "simple_apm_rgbd_mid360.sdf"
LANDMARKS = (
    PACKAGE_ROOT / "models" / "s_curve_lidar_landmarks" / "model.sdf"
)
AIRCRAFT_MODEL = PACKAGE_ROOT / "models" / "iris_apm_rgbd" / "model.sdf"


def pose_xy(element):
    values = [float(value) for value in element.findtext("pose").split()]
    return values[0], values[1]


def named_children(root, tag, prefix):
    return [
        element for element in root.findall(f".//{tag}")
        if element.get("name", "").startswith(prefix)
    ]


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
