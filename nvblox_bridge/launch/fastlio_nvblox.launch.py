"""
Launch file for running FAST-LIO with nvblox voxel reconstruction.

This launches:
1. FAST-LIO for LiDAR-inertial odometry
2. nvblox_fastlio_bridge for real-time voxel reconstruction
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    fastlio_share = get_package_share_directory('fast_lio')
    nvblox_bridge_share = get_package_share_directory('nvblox_fastlio_bridge')

    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='ouster32',
        description='FAST-LIO config file name (without .yaml)'
    )

    # FAST-LIO launch
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(fastlio_share, 'launch', 'mapping.launch.py')
        ]),
        launch_arguments={
            'config': LaunchConfiguration('config'),
            'rviz': 'false'  # We'll use our own RViz config
        }.items()
    )

    # nvblox bridge node
    nvblox_bridge_node = Node(
        package='nvblox_fastlio_bridge',
        executable='nvblox_fastlio_node',
        name='nvblox_fastlio_node',
        output='screen',
        parameters=[
            os.path.join(nvblox_bridge_share, 'config', 'nvblox_params.yaml')
        ],
        remappings=[
            ('/Odometry', '/Odometry'),
            ('/cloud_registered', '/cloud_registered')
        ]
    )

    # RViz with custom config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nvblox_bridge_share, 'config', 'nvblox_fastlio.rviz')],
        condition=LaunchConfiguration('rviz')
    )

    return LaunchDescription([
        rviz_arg,
        config_arg,
        fastlio_launch,
        nvblox_bridge_node,
        # rviz_node,  # Uncomment when RViz config is created
    ])
