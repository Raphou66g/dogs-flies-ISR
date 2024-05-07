import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory("unitree_ros")
    default_param_file = os.path.join(pkg_dir, "config", "params.yaml")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_param_file),
        description="Parameters file to be used.",
    )
    use_wifi_arg = DeclareLaunchArgument(
        "wifi",
        default_value="false",
        description="Uses the wifi IP for communicating with the robot",
    )

    return LaunchDescription(
        [params_file_arg, use_wifi_arg, OpaqueFunction(function=launch_unitree_driver)]
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
