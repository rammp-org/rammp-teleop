from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("rammp_teleop"), "config", "space_mouse.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="params for space_teleop node",
            ),
            Node(
                package="spacenav",
                executable="spacenav_node",
                name="spacenav_node",
                output="screen",
            ),
            Node(
                package="rammp_teleop",
                executable="space_teleop",
                name="space_teleop",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
