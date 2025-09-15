#!/usr/bin/env python3
"""
로봇 제어 GUI 런치 파일
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 로봇 제어 GUI 노드
        Node(
            package='ros_robot_control_gui',
            executable='robot_control_gui.py',
            name='robot_control_gui',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
    ])
