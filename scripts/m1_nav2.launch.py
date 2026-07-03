"""M1 Nav2 导航 launch — 直接 cmd_vel，限速靠 Nav2 params + SDK 低速档
"""
import os
import time
import subprocess
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    params_file = os.path.join(pkg_dir, "config", "nav2_params.yaml")

    body_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "body", "base_link"],  # identity
        name="body_to_base_link_tf",
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file],
        
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
        
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file],
        
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file],
        
    )

    # 手动激活脚本：等所有节点就绪后顺序激活
    def activate_nodes(context, *args, **kwargs):
        import subprocess, time
        source = "source /opt/ros/humble/setup.bash"
        nodes = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
        for node in nodes:
            time.sleep(3)
            cmd = f"bash -c '{source} && ros2 lifecycle set /{node} configure && ros2 lifecycle set /{node} activate'"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
            print(f"[ACTIVATE] {node}: {result.stdout.strip()}")
            if result.returncode != 0:
                print(f"[ACTIVATE] ERROR {node}: rc={result.returncode}, stderr={result.stderr.strip()}")

        # 导航前自动设 SDK 低速档
        time.sleep(2)
        cmd = f"bash -c '{source} && ros2 service call /set_speed_low std_srvs/srv/Trigger'"
        subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        print("[ACTIVATE] SDK speed set to LOW")

    activator = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=activate_nodes)],
    )

    return LaunchDescription([
        body_to_base_link_tf,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        activator,
    ])
