from glob import glob
from setuptools import find_packages, setup

package_name = "uf_reliability"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zyc",
    maintainer_email="zyc@example.com",
    description="Independent sensor degradation scores for Ultra-Fusion simulation.",
    license="MIT",
    entry_points={"console_scripts": [
        "reliability_monitor = uf_reliability.reliability_monitor:main",
        "reliability_scheduler = uf_reliability.reliability_scheduler:main",
        "relocalization_risk_shadow = "
        "uf_reliability.relocalization_risk_shadow_node:main",
        "relocalization_trigger_shadow_matrix = "
        "uf_reliability.relocalization_trigger_shadow_matrix:main",
    ]},
)
