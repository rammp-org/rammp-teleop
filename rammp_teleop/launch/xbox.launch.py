"""joy_node + xbox_teleop. Run against an already-running kinova_gen3_node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("rammp_teleop"), "config", "xbox.yaml"
    )
    params_file = LaunchConfiguration("params_file")
    controller = LaunchConfiguration("controller")
    device_id = LaunchConfiguration("device_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "controller",
                default_value="ee_twist",
                description="ee_twist | joint_velocity",
            ),
            DeclareLaunchArgument(
                "device_id", default_value="0", description="/dev/input/js<N>"
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[params_file, {"device_id": device_id}],
                output="screen",
            ),
            Node(
                package="rammp_teleop",
                executable="xbox_teleop",
                name="xbox_teleop",
                parameters=[params_file, {"controller": controller}],
                output="screen",
            ),
        ]
    )
