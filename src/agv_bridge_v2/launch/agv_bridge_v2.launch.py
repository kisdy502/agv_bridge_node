#!/usr/bin/env python3
"""
AGV Bridge Launch File
用于启动ROS2与Spring Boot的桥梁节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 定义启动参数
    agv_id_arg = DeclareLaunchArgument(
        "agv_id", default_value="AGV001", description="AGV的唯一标识符"
    )

    springboot_host_arg = DeclareLaunchArgument(
        "springboot_host",
        default_value="192.168.2.130",
        description="Spring Boot服务器地址",
    )

    springboot_port_arg = DeclareLaunchArgument(
        "springboot_port", default_value="22777", description="Spring Boot服务器端口"
    )

    websocket_port_arg = DeclareLaunchArgument(
        "websocket_port", default_value="9090", description="WebSocket服务器端口"
    )

    # 创建节点
    agv_bridge_node = Node(
        package="agv_bridge_v2",
        executable="agv_bridge_node",
        name="agv_bridge_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                "agv_id": LaunchConfiguration("agv_id"),
                "springboot_host": LaunchConfiguration("springboot_host"),
                "springboot_port": LaunchConfiguration("springboot_port"),
                "websocket_port": LaunchConfiguration("websocket_port"),
            }
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            agv_id_arg,
            springboot_host_arg,
            springboot_port_arg,
            websocket_port_arg,
            agv_bridge_node,
        ]
    )
