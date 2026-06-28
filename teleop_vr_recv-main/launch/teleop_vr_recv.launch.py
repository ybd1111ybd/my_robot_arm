#!/usr/bin/env python3
"""
Launch file for teleop_vr_recv_node.

Usage:
    ros2 launch teleop_vr_recv teleop_vr_recv.launch.py
    ros2 launch teleop_vr_recv teleop_vr_recv.launch.py port:=8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for teleop_vr_recv_node."""
    # Declare launch arguments
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Robot namespace for multi-robot systems'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='UDP port to listen on'
    )

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='UDP host address to bind'
    )

    enable_udp_receive_arg = DeclareLaunchArgument(
        'enable_udp_receive',
        default_value='false',
        description='Enable UDP receive on startup (default: false)'
    )

    # Create the node
    teleop_vr_recv_node = Node(
        package='teleop_vr_recv',
        executable='teleop_vr_recv_node',
        name='teleop_vr_recv_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'host': LaunchConfiguration('host'),
            'enable_udp_receive': LaunchConfiguration('enable_udp_receive'),
        }],
        respawn=False,
    )

    return LaunchDescription([
        robot_namespace_arg,
        port_arg,
        host_arg,
        enable_udp_receive_arg,
        teleop_vr_recv_node,
    ])
