import math
import xml.etree.ElementTree as ET
from pathlib import Path

from multi_slam_uav_sim.s_curve_path import generate_s_curve


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD = PACKAGE_ROOT / "worlds" / "simple_apm_rgbd_mid360.sdf"
LANDMARKS = (
    PACKAGE_ROOT / "models" / "s_curve_lidar_landmarks" / "model.sdf"
)


def pose_xy(element):
    values = [float(value) for value in element.findtext("pose").split()]
    return values[0], values[1]


def named_children(root, tag, prefix):
    return [
        element for element in root.findall(f".//{tag}")
        if element.get("name", "").startswith(prefix)
    ]


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
    assert all(
        max(float(value) for value in item.findtext("geometry/box/size").split())
        >= 38.0
        for item in x_lines + y_lines
    )

    flow_markers = named_children(root, "visual", "marker_")
    assert len(flow_markers) >= 32
    assert max(max(abs(value) for value in pose_xy(item)) for item in flow_markers) >= 15.0


def test_lidar_landmarks_cover_the_s_curve_and_outer_area():
    root = ET.parse(LANDMARKS).getroot()
    landmarks = root.findall(".//collision")
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
