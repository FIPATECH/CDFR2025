import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Récupération des dossiers partagés
    pkg_nav2    = get_package_share_directory('nav2_bringup')
    pkg_self    = get_package_share_directory('robot_navigation')
    pkg_desc    = get_package_share_directory('robot_description')
    pkg_rplidar = get_package_share_directory('rplidar_ros2')

    # Déclarations d'arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_self, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the nav2 stack'
    )

    # Substitutions pour réutilisation
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    autostart    = LaunchConfiguration('autostart')

    # Robot_state_publisher (lecture directe du URDF)
    urdf_path = Path(pkg_desc) / 'urdf' / 'robot.urdf'
    robot_description_content = urdf_path.read_text()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time':      use_sim_time
        }]
    )

    # Map Server pour le keepout mask
    mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_keepout',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'yaml_filename': os.path.join(pkg_self, 'config', 'keepout_mask.yaml'),
            'use_sim_time':  False,
        }],
        remappings=[('/map', '/keepout_mask')],
    )
    lifecycle_keepout = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names':   ['map_server_keepout']
        }]
    )

    # Driver RPLIDAR
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar.launch.py')
        ),
        launch_arguments={
            'serial_port':     '/dev/ttyUSB0',
            'serial_baudrate': '115200',
            'frame_id':        'laser_frame',
        }.items()
    )
    
    tf_laser_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_frame_alias',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'lidar_link',
            'laser_frame'
        ]
    )


    # Visualisation RPLIDAR (RViz)
    view_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'view_rplidar.launch.py')
        ),
        launch_arguments={}.items()
    )

    # Nav2 bringup avec map codée en dur
    fixed_map_path = os.path.join(
        pkg_self, 'config', 'table_map.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  params_file,
            'autostart':    autostart,
            'map':          fixed_map_path,
        }.items()
    )

    # TF static transforms
    tf_foot_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_foot_to_link',
        arguments=['0','0','0','0','0','0','base_footprint','base_link']
    )
    tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_footprint',
        arguments=['0','0','0','0','0','0','odom','base_footprint']
    )

    # Les nœuds applicatifs
    uart_bridge      = Node(
        package='uart_bridge',
        executable='uart_bridge_node',
        name='uart_bridge',
        output='screen'
    )
    match_control    = Node(
        package='match_control',
        executable='match_control_node',
        name='match_control',
        output='screen'
    )
    robot_navigation = Node(
        package='robot_navigation',
        executable='robot_navigation_node',
        name='robot_navigation',
        output='screen'
    )

    # Assemblage final
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,

        robot_state_publisher,
        mask_server,
        lifecycle_keepout,

        rplidar_launch,
        tf_laser_alias,
        view_rplidar,

        nav2,

        tf_foot_to_link,
        tf_odom,

        uart_bridge,
        match_control,
        robot_navigation,
    ])
