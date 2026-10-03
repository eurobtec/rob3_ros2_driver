from setuptools import find_packages, setup
import os
from glob import glob

package_name = "rob3_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml") + glob("config/*.rviz")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
        # install the demo shell script where `ros2 run` can find+exec it
        (os.path.join("lib", package_name), glob("scripts/*.sh")),
    ],
    install_requires=["setuptools", "rob3"],
    zip_safe=True,
    maintainer="ROB3 project",
    maintainer_email="dev@example.com",
    description="ROS 2 driver for the Eurobtec ROB 3 over RS-232 (Python).",
    license="See repository LICENSE",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rob3_driver_node = rob3_driver.rob3_driver_node:main",
            "rob3_jog_keyboard = rob3_driver.jog_keyboard:main",
            "rob3_fake_robot = rob3.fake_robot:main",
        ],
    },
)
