from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='match_control',
            executable='match_control_node',
            name='match_control',
            output='screen'
        )
    ])


