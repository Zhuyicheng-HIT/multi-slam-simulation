from glob import glob
from setuptools import find_packages, setup

package_name = "uf_shared_mapping"
setup(name=package_name, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"]),
                  ("share/" + package_name + "/config", glob("config/*.yaml")),
                  ("share/" + package_name + "/launch", glob("launch/*.launch.py"))],
      install_requires=["setuptools"], zip_safe=True, maintainer="Yicheng-Zhu",
      maintainer_email="2025111580@stu.hit.edu.cn",
      description="Source-aware online shared map", license="Apache-2.0",
      entry_points={"console_scripts": ["shared_mapping = uf_shared_mapping.shared_mapping_node:main"]})
