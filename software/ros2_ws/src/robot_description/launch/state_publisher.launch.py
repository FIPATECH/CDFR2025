import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_desc = FindPackageShare('robot_description').find('robot_description')
    urdf_path = PathJoinSubstitution([pkg_desc, 'urdf', 'robot.urdf'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Publisher URDF
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': urdf_path}
        ]
    )

    # Inclusion de Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('nav2_bringup').find('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(
                FindPackageShare('robot_navigation').find('robot_navigation'),
                'config', 'nav2_params.yaml'
            ),
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('model', default_value=str(urdf_path)),
        rsp,
        nav2,
    ])
