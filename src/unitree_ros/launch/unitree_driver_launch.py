import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    os.path.join(
                        get_package_share_directory("unitree_ros"),
                        "config",
                        "params.yaml",
                    )
                ),
                description="Parameters file to be used.",
            ),
            DeclareLaunchArgument(
                "wifi",
                default_value="false",
                description="Uses the wifi IP for communicating with the robot",
            ),
            OpaqueFunction(function=launch_unitree_driver),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="false",
                choices=["true", "false"],
                description="Open RVIZ for Go1 visualization",
            ),
            DeclareLaunchArgument(
                name="fixed_frame",
                default_value="base_footprint",
                description="Fixed frame for RVIZ",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("go1_description"),
                            "launch",
                            "load_go1.launch.py",
                        ]
                    )
                ),
                launch_arguments=[
                    ("use_jsp", "none"),
                    ("fixed_frame", LaunchConfiguration("fixed_frame")),
                    ("use_nav2_links", "true"),
                    ("use_rviz", LaunchConfiguration("use_rviz")),
                ],
            ),
        ]
    )


def launch_unitree_driver(context):
    params_file = LaunchConfiguration("params_file")
    wifi = context.launch_configurations.get("wifi", "false")
    if wifi == "true":
        robot_ip = "192.168.12.1"
    else:
        robot_ip = "192.168.123.161"

    unitree_driver_node = Node(
        package="unitree_ros",
        executable="unitree_driver",
        parameters=[params_file, {"robot_ip": robot_ip}],
        output="screen",
    )

    return [unitree_driver_node]
