from glob import glob
from setuptools import find_packages, setup


package_name = "uf_lio_adapter"

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
    description="Stable LIO namespace and external diagnostic proxy.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lio_adapter = uf_lio_adapter.lio_adapter:main",
            "native_factor_validator = uf_lio_adapter.native_factor_validator:main",
        ]
    },
)
