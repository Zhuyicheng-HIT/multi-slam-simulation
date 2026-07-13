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
            "fault_injector = uf_sensor_pipeline.fault_injector:main",
            "pointcloud_body_filter = uf_sensor_pipeline.pointcloud_body_filter:main",
            "sensor_contract_monitor = uf_sensor_pipeline.sensor_contract_monitor:main",
        ]
    },
)
