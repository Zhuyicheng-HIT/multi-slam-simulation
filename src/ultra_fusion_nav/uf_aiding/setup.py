from setuptools import find_packages, setup


package_name = "uf_aiding"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zyc",
    maintainer_email="zyc@example.com",
    description="Conservative aiding admission and smooth re-anchor logic.",
    license="MIT",
)
