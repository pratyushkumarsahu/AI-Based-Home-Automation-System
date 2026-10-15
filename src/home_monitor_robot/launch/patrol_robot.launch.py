#!/usr/bin/env python3
"""Launch patrol robot"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.expanduser(
        '~/home_monitor_ws/src/home_monitor_robot/urdf/patrol_robot.urdf'
    )
    
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc}
            ]
        ),
        
        Node(
            package='home_monitor_robot',
            executable='patrol_robot',
            name='patrol_robot',
            output='screen'
        ),
    ])
