from setuptools import find_packages, setup
import os
from glob import glob

package_name = "sonair_evk"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # During installation, we need to copy the launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "pointcloud = sonair_evk.point_cloud_publisher:main",
            "transform_publisher = sonair_evk.transform_publisher:main",
        ],
    },
)
