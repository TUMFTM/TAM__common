import setuptools
from pathlib import Path
from glob import glob
import os

package_name = "ros2_watchdog_py"
version = "0.0.1"

with open(Path(__file__).parent.joinpath("requirements.txt")) as f:
    requirements = f.read().splitlines()

# Filter requirement string
requirements = [
    requirement.split("#")[0].strip()
    for requirement in requirements
    if "--extra-index-url" not in requirement
    and len(requirement.split("#")[0]) > 0
    and "../" not in requirement
]

setuptools.setup(
    name=package_name,
    version=version,
    packages=setuptools.find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", package_name, "src"), glob("ros_watchdog_py/*.py")),
    ],
    install_requires=requirements,
    zip_safe=True,
    maintainer="Dominic Ebner",
    maintainer_email="dominic.ebner@tum.de",
    description="Generic watchdog functionality for Python ROS 2 nodes",
    license="Apache 2.0",
    tests_require=requirements + ["pytest"],
    entry_points={
        "console_scripts": ["test_node = ros2_watchdog_py.monitor_example:main"],
    },
)
