import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rammp_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Swapnil Pande",
    maintainer_email="swapnil.pande98@gmail.com",
    description="Xbox and Quest 3 teleop for the Kinova Gen3 via the streaming tier.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            f"xbox_teleop = {package_name}.xbox_node:main",
            f"quest_teleop = {package_name}.quest_node:main",
            f"quest_reader = {package_name}.quest_reader:main",
            f"quest_calibrate = {package_name}.quest.calibrate:main",
        ],
    },
)
