from glob import glob
from setuptools import find_packages, setup


package_name = "uf_sensor_pipeline"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zyc",
    maintainer_email="zyc@example.com",
    description="Ultra-Fusion sensor normalization and controlled fault injection.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "external_nav_gate = uf_sensor_pipeline.external_nav_gate:main",
            "fcu_observation_bridge = uf_sensor_pipeline.fcu_observation_bridge:main",
            "fault_injector = uf_sensor_pipeline.fault_injector:main",
            "sensor_relay_manager = uf_sensor_pipeline.sensor_relay_manager:main",
            "robustness_fault_injector = uf_sensor_pipeline.robustness_fault_injector:main",
            "gps_flow_fusion = uf_sensor_pipeline.gps_flow_fusion_node:main",
            "gnss_metadata_relay = uf_sensor_pipeline.gnss_metadata_relay:main",
            "nmea_gnss = uf_sensor_pipeline.nmea_gnss_node:main",
            "pointcloud_body_filter = uf_sensor_pipeline.pointcloud_body_filter:main",
            "sensor_contract_monitor = uf_sensor_pipeline.sensor_contract_monitor:main",
        ]
    },
)
