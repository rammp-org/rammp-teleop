"""quest_reader + quest_teleop. Run against an already-running kinova_gen3_node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("rammp_teleop"), "config", "quest.yaml"
    )
    params_file = LaunchConfiguration("params_file")
    controller = LaunchConfiguration("controller")
    hand = LaunchConfiguration("hand")
    quest_ip = LaunchConfiguration("quest_ip")
    mock = LaunchConfiguration("mock")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "controller",
                default_value="ee_pose_impedance",
                description="ee_pose_impedance | ee_pose_position",
            ),
            DeclareLaunchArgument(
                "hand", default_value="right", description="right | left"
            ),
            DeclareLaunchArgument(
                "quest_ip", default_value="", description="adb over Wi-Fi; empty = USB"
            ),
            DeclareLaunchArgument(
                "mock",
                default_value="false",
                description="scripted controller, no headset",
            ),
            Node(
                package="rammp_teleop",
                executable="quest_reader",
                name="quest_reader",
                parameters=[
                    params_file,
                    {"hand": hand, "quest_ip": quest_ip, "mock": mock},
                ],
                output="screen",
            ),
            Node(
                package="rammp_teleop",
                executable="quest_teleop",
                name="quest_teleop",
                parameters=[params_file, {"controller": controller, "hand": hand}],
                output="screen",
            ),
        ]
    )
