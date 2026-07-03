"""
Launch file for running FAST-LIO with nvblox voxel reconstruction.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    nvblox_bridge_share = get_package_share_directory('nvblox_fastlio_bridge')
    fast_lio_share = get_package_share_directory('fast_lio')
    
    # Launch arguments
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz'
    )

    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2',
        default_value='false',
        description='Launch Nav2'
    )
    
    # Keep the original argument name for backward compatibility, but prefer bridge_config.
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(nvblox_bridge_share, 'config', 'nvblox_fastlio.yaml'),
        description='[DEPRECATED] Use bridge_config instead'
    )

    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value=LaunchConfiguration('config_file'),
        description='Path to the nvblox_fastlio_bridge parameter file'
    )
    
    fastlio_config_arg = DeclareLaunchArgument(
        'fastlio_config',
        default_value=os.path.join(fast_lio_share, 'config', 'ouster32.yaml'),
        description='Absolute path to the FAST-LIO parameter file'
    )

    bridge_params = ParameterFile(LaunchConfiguration('bridge_config'))
    
    # Include FAST-LIO launch
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('fastlio_config'),
            'rviz': 'false'  # We'll launch our own RViz
        }.items()
    )
    
    # nvblox bridge node
    nvblox_node = Node(
        package='nvblox_fastlio_bridge',
        executable='nvblox_fastlio_node',
        name='nvblox_fastlio_node',
        output='screen',
        parameters=[bridge_params],
    )

    # ESDF to Grid Converter
    esdf_converter_node = Node(
        package='nvblox_fastlio_bridge',
        executable='esdf_to_grid',
        name='esdf_to_grid',
        output='screen',
        parameters=[{'resolution': 0.1, 'max_dist': 2.0}]
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nvblox_bridge_share, 'launch', 'nav2.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_nav2'))
    )
    
    # RViz
    rviz_config_path = os.path.join(nvblox_bridge_share, 'rviz', 'nvblox_mapper.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    return LaunchDescription([
        launch_rviz_arg,
        launch_nav2_arg,
        config_file_arg,
        bridge_config_arg,
        fastlio_config_arg,
        fast_lio_launch,
        nvblox_node,
        esdf_converter_node,
        nav2_launch,
        rviz_node,
    ])
