from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    uart_bridge = Node(
        package='uart_bridge',
        executable='uart_bridge_node',
        name='uart_bridge',
        output='screen'
    )
    
    match_control = Node(
        package='match_control',
        executable='match_control_node',
        name='match_control',
        output='screen'
    )

    return LaunchDescription([
        uart_bridge,
        match_control
    ])
