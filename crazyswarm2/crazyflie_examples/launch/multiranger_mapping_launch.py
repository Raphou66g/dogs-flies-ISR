import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    server_params = crazyflies

    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    with open(urdf, 'r') as f:
        robot_desc = f.read()
    server_params['robot_description'] = robot_desc

    return LaunchDescription([
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=[server_params],
        ),
        Node(
            package='crazyflie',
            executable='vel_mux.py',
            name='vel_mux',
            output='screen',
            parameters=[{'hover_height': 0.3},
                        {'incoming_twist_topic': '/cmd_vel'},
                        {'robot_prefix': '/cf231'}]
        ),
        Node(
            parameters=[
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'base_frame': 'cf231'},
                {'scan_topic': '/cf231/scan'},
                {'use_scan_matching': False},
                {'max_laser_range': 3.5},
                {'resolution': 0.1},
                {'minimum_travel_distance': 0.01},
                {'minimum_travel_heading': 0.001},
                {'map_update_interval': 0.1}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'),
    ])
