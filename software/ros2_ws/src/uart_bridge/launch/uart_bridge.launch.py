from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uart_bridge',
            executable='uart_bridge_node',
            name='uart_bridge',
            output='screen'
        )
    ])
