#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config',
        'slam_toolbox_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        )
    ])
