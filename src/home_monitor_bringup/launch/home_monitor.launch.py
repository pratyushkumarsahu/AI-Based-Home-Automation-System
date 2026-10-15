#!/usr/bin/env python3
"""Launch file for complete home monitoring system"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get world file path
    world_file = os.path.expanduser(
        '~/home_monitor_ws/src/home_monitor_gazebo/worlds/house.sdf'
    )
    
    return LaunchDescription([
        # Gazebo simulation (using gz command directly)
        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '-v', '4'],
            output='screen',
            name='gazebo'
        ),
        
        # Sensor nodes
        Node(
            package='home_monitor_sensors',
            executable='camera_node',
            name='camera_sim',
            output='screen'
        ),
        
        Node(
            package='home_monitor_sensors',
            executable='motion_sensor_node',
            name='motion_sensor',
            output='screen'
        ),
        
        # Core monitoring
        Node(
            package='home_monitor_core',
            executable='monitoring_node',
            name='home_monitor',
            output='screen'
        ),
        
        # Perception/AI
        Node(
            package='home_monitor_perception',
            executable='detection_node',
            name='ai_detector',
            output='screen'
        ),
    ])
