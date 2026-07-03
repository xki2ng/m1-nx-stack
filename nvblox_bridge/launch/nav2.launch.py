import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    nvblox_bridge_share = get_package_share_directory('nvblox_fastlio_bridge')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nvblox_bridge_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    # Static transforms to link FAST-LIO frames to Nav2 frames
    # map -> odom -> camera_init -> body -> base_link
    
    # map -> odom (Identity)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # odom -> camera_init (Identity) - assuming FAST-LIO publishes camera_init -> body
    odom_to_camera_init_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_camera_init_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init']
    )

    # body -> base_link (Identity) - assuming FAST-LIO publishes camera_init -> body
    body_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link']
    )

    # Launch Nav2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart}.items())

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    
    ld.add_action(map_to_odom_tf)
    ld.add_action(odom_to_camera_init_tf)
    ld.add_action(body_to_base_link_tf)
    ld.add_action(nav2_bringup_launch)

    return ld
