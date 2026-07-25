from glob import glob
from setuptools import find_packages, setup

package_name = "uf_backend_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="zyc",
    maintainer_email="zyc@example.com",
    description="Scheduler-weighted online and offline sliding-window fusion prototype.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "run_backend_ablation = uf_backend_fusion.ablation:main",
            "extract_backend_factors = uf_backend_fusion.bag_cli:extract_main",
            "replay_backend_factors = uf_backend_fusion.bag_cli:replay_main",
            "online_backend_fusion = uf_backend_fusion.online_backend:main",
        ],
    },
)
