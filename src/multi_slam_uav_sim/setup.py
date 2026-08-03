from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "multi_slam_uav_sim"


def files_under(root):
    return [str(p) for p in Path(root).rglob("*") if p.is_file()]


data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/worlds", glob("worlds/*.sdf") + glob("worlds/*.world")),
    (f"share/{package_name}/config", glob("config/*")),
    (f"share/{package_name}/params", glob("params/*")),
    (f"share/{package_name}/scripts", glob("scripts/*")),
]

for model_dir in sorted(Path("models").iterdir()):
    if model_dir.is_dir():
        for subdir in sorted({p.parent for p in model_dir.rglob("*") if p.is_file()}):
            rel = subdir.as_posix()
            data_files.append((f"share/{package_name}/{rel}", files_under(subdir)))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zyc",
    maintainer_email="zyc@example.com",
    description="APM UAV sensor simulation and MAVROS-compatible bridges for multi-slam.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "d435i_sim_bridge = multi_slam_uav_sim.d435i_sim_bridge:main",
            "external_nav_accuracy = multi_slam_uav_sim.external_nav_accuracy:main",
            "fcu_mavlink_flow_receiver = multi_slam_uav_sim.fcu_mavlink_flow_receiver:main",
            "flight_state_bridge = multi_slam_uav_sim.flight_state_bridge:main",
            "gazebo_clock_bridge = multi_slam_uav_sim.gazebo_clock_bridge:main",
            "gazebo_optical_flow_to_mavros = multi_slam_uav_sim.gazebo_optical_flow_to_mavros:main",
            "gz_mid360_pointcloud_bridge = multi_slam_uav_sim.gz_mid360_pointcloud_bridge:main",
            "gz_rgbd_latest_bridge = multi_slam_uav_sim.gz_rgbd_latest_bridge:main",
            "guided_flight = multi_slam_uav_sim.guided_flight:main",
            "guided_rectangle_waypoints = multi_slam_uav_sim.guided_rectangle_waypoints:main",
            "guided_s_curve_waypoints = multi_slam_uav_sim.guided_s_curve_waypoints:main",
            "livox_mid360_bridge = multi_slam_uav_sim.livox_mid360_bridge:main",
            "flow_gazebo_accuracy = multi_slam_uav_sim.flow_gazebo_accuracy:main",
            "mavros_stream_requester = multi_slam_uav_sim.mavros_stream_requester:main",
            "mtf01_micolink_bridge = multi_slam_uav_sim.mtf01_micolink_bridge:main",
            "mtf01p_mavlink_bridge = multi_slam_uav_sim.mtf01p_mavlink_bridge:main",
            "mtf01p_mavlink_sensor = multi_slam_uav_sim.mtf01p_mavlink_sensor:main",
            "optical_flow_viewer = multi_slam_uav_sim.optical_flow_viewer:main",
            "rectangle_flow_test = multi_slam_uav_sim.rectangle_flow_test:main",
            "people_motion = multi_slam_uav_sim.people_motion:main",
            "rgbd_camera_follow = multi_slam_uav_sim.rgbd_camera_follow:main",
            "simulation_performance_monitor = multi_slam_uav_sim.simulation_performance_monitor:main",
        ]
    },
)
