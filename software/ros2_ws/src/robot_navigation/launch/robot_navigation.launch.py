import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_self = get_package_share_directory('robot_navigation')

    # 1) Nav2 bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': os.path.join(pkg_self, 'config', 'table_map.yaml'),
            'params_file': os.path.join(pkg_self, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    # 2) slam_toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(pkg_self, 'config', 'slam_toolbox_params.yaml'),
            'use_sim_time': 'false'
        }.items()
    )

    # 3) Votre nœud robot_navigation
    robot_nav = Node(
        package='robot_navigation',
        executable='robot_navigation_node',
        name='robot_navigation',
        output='screen',
        parameters=[{
            # si vous aviez des params à injecter, sinon enlevez
        }]
    )

    return LaunchDescription([
        nav2,
        slam,
        robot_nav,
    ])
