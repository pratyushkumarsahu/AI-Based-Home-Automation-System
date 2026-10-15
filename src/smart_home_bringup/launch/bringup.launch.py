#!/usr/bin/env python3
"""
Main Bringup Launch File for AI-Based Smart Home Monitoring System

Usage:
  ros2 launch smart_home_bringup bringup.launch.py \
    headless:=true/false \
    use_rviz:=true/false \
    use_nav2:=true/false \
    scenario:=patrol|obstacle|intruder|family|gas_leak|full \
    gui_view:=default|topdown|follow \
    log_data:=true/false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    headless_str = LaunchConfiguration('headless').perform(context).lower()
    is_headless = headless_str in ['true', '1', 'yes']
    use_rviz_str = LaunchConfiguration('use_rviz').perform(context).lower()
    use_rviz = use_rviz_str in ['true', '1', 'yes']
    scenario = LaunchConfiguration('scenario').perform(context).lower()
    use_nav2_str = LaunchConfiguration('use_nav2').perform(context).lower()
    use_nav2 = use_nav2_str in ['true', '1', 'yes']
    gui_view = LaunchConfiguration('gui_view').perform(context).lower()
    log_data_str = LaunchConfiguration('log_data').perform(context).lower()
    log_data = log_data_str in ['true', '1', 'yes']

    # Package directories
    bringup_pkg = get_package_share_directory('smart_home_bringup')
    gazebo_pkg = get_package_share_directory('smart_home_gazebo')
    desc_pkg = get_package_share_directory('smart_home_description')

    world_path = os.path.join(gazebo_pkg, 'worlds', 'house.sdf')
    bridge_config = os.path.join(gazebo_pkg, 'config', 'bridge_params.yaml')
    params_file = os.path.join(bringup_pkg, 'config', 'params.yaml')
    rviz_config = os.path.join(bringup_pkg, 'config', 'home_monitor.rviz')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'patrol_robot.urdf.xacro')

    # Process URDF xacro
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # Select GUI config for Gazebo Harmonic
    gui_config_file = os.path.join(gazebo_pkg, 'config', 'gui.config')
    if gui_view == 'topdown':
        gui_config_file = os.path.join(gazebo_pkg, 'config', 'gui_topdown.config')
    elif gui_view == 'follow':
        gui_config_file = os.path.join(gazebo_pkg, 'config', 'gui_follow.config')

    # Gazebo simulation process
    gz_args = [world_path, '-r']
    if is_headless:
        gz_args = ['-s'] + gz_args
    else:
        if os.path.exists(gui_config_file):
            gz_args += ['--gui-config', gui_config_file]

    gz_env = os.environ.copy()
    gz_env['LIBGL_ALWAYS_SOFTWARE'] = os.environ.get('LIBGL_ALWAYS_SOFTWARE', '1')
    gz_env['MESA_GL_VERSION_OVERRIDE'] = os.environ.get('MESA_GL_VERSION_OVERRIDE', '3.3')

    gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim'] + gz_args,
        output='screen',
        additional_env=gz_env
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn Robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'smart_home_world',
            '-name', 'patrol_robot',
            '-string', doc.toxml(),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
            '-Y', '0.0'
        ]
    )

    # ROS-GZ Parameter Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }]
    )

    actions = [
        gazebo_cmd,
        robot_state_pub,
        bridge_node,
        TimerAction(period=3.0, actions=[spawn_robot])
    ]

    # Model paths
    intruder_sdf = os.path.join(gazebo_pkg, 'models', 'intruder', 'model.sdf')
    box_sdf = os.path.join(gazebo_pkg, 'models', 'obstacle_box', 'model.sdf')
    pratyush_sdf = os.path.join(gazebo_pkg, 'models', 'family_pratyush', 'model.sdf')
    arpita_sdf = os.path.join(gazebo_pkg, 'models', 'family_arpita', 'model.sdf')
    rohit_sdf = os.path.join(gazebo_pkg, 'models', 'family_rohit', 'model.sdf')

    spawn_box_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'smart_home_world',
            '-name', 'obstacle_box',
            '-file', box_sdf,
            '-x', '1.0',
            '-y', '-0.8',
            '-z', '0.15',
            '-Y', '0.0'
        ]
    )

    spawn_pratyush_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'smart_home_world',
            '-name', 'family_pratyush',
            '-file', pratyush_sdf,
            '-x', '2.5',
            '-y', '-0.5',
            '-z', '0.0',
            '-Y', '3.14'
        ]
    )

    spawn_arpita_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'smart_home_world',
            '-name', 'family_arpita',
            '-file', arpita_sdf,
            '-x', '1.5',
            '-y', '-0.7',
            '-z', '0.0',
            '-Y', '3.14'
        ]
    )

    spawn_rohit_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'smart_home_world',
            '-name', 'family_rohit',
            '-file', rohit_sdf,
            '-x', '1.8',
            '-y', '0.8',
            '-z', '0.0',
            '-Y', '1.57'
        ]
    )

    # Intruder spawn coordinates:
    # In intruder scenario: in front of robot in living room approach
    # In full scenario: at hallway junction entering patrol corridor
    intruder_x = '2.2' if scenario == 'intruder' else '2.0'
    intruder_y = '-0.8' if scenario == 'intruder' else '0.0'
    spawn_intruder_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'smart_home_world',
            '-name', 'intruder',
            '-file', intruder_sdf,
            '-x', intruder_x,
            '-y', intruder_y,
            '-z', '0.0',
            '-Y', '3.14'
        ]
    )

    # Gas leak active flag by scenario
    gas_leak_active = scenario in ['gas_leak', 'full']

    # Scenario Spawning Logic
    if scenario == 'intruder':
        actions.append(TimerAction(period=3.5, actions=[spawn_intruder_node]))
    elif scenario == 'obstacle':
        actions.append(TimerAction(period=3.5, actions=[spawn_box_node]))
    elif scenario == 'family':
        actions.append(TimerAction(period=3.5, actions=[
            spawn_pratyush_node, spawn_arpita_node, spawn_rohit_node
        ]))
    elif scenario == 'gas_leak':
        gas_trigger_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/gas_leak/trigger', 'std_msgs/msg/Bool', '{data: true}'],
            output='screen'
        )
        actions.append(TimerAction(period=4.0, actions=[gas_trigger_cmd]))
    elif scenario == 'full':
        # Full scenario: All 3 family members + obstacle box spawned at 3.5s
        actions.append(TimerAction(period=3.5, actions=[
            spawn_pratyush_node, spawn_arpita_node, spawn_rohit_node, spawn_box_node
        ]))
        # Intruder arrives at 6.0s
        actions.append(TimerAction(period=6.0, actions=[spawn_intruder_node]))
        # Gas leak activates at 8.0s
        gas_trigger_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/gas_leak/trigger', 'std_msgs/msg/Bool', '{data: true}'],
            output='screen'
        )
        actions.append(TimerAction(period=8.0, actions=[gas_trigger_cmd]))

    # Person locomotion motion node (walking routes in Gazebo)
    if scenario in ['family', 'intruder', 'full']:
        person_motion_node = Node(
            package='smart_home_gazebo',
            executable='person_motion_node',
            output='screen',
            parameters=[{'scenario': scenario, 'use_sim_time': True}]
        )
        actions.append(TimerAction(period=4.5, actions=[person_motion_node]))

    # Core C++ Nodes
    # 1. Navigation Node (Custom Waypoint Follower or Nav2)
    if not use_nav2:
        nav_node = Node(
            package='smart_home_navigation',
            executable='patrol_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}]
        )
        actions.append(nav_node)

    # 2. AI Detection Node
    detection_node = Node(
        package='smart_home_detection',
        executable='detection_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )
    actions.append(detection_node)

    # 3. Decision Maker State Machine Node
    decision_node = Node(
        package='smart_home_decision',
        executable='decision_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )
    actions.append(decision_node)

    # 4. Alert Manager Node
    alert_node = Node(
        package='smart_home_alerts',
        executable='alert_manager_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )
    actions.append(alert_node)

    # 5. Gas Sensor Node (with visual plume emitter, fallback cloud, and RViz markers)
    gas_sensor_node = Node(
        package='smart_home_gazebo',
        executable='gas_sensor_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': True,
            'leak_active': gas_leak_active
        }]
    )
    actions.append(gas_sensor_node)

    # 6. Data Logger Node (P4: writes CSV logs and events.jsonl to results/<scenario>_<timestamp>/)
    if log_data:
        record_script = os.path.expanduser('~/home_monitor_ws/scripts/record_data.py')
        if os.path.exists(record_script):
            record_cmd = ExecuteProcess(
                cmd=['python3', record_script, scenario],
                output='screen'
            )
            actions.append(TimerAction(period=2.0, actions=[record_cmd]))

    # 7. RViz2 (optional)
    if use_rviz:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        )
        actions.append(rviz_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode without GUI (-s)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Use Nav2 stack instead of custom waypoint follower'
        ),
        DeclareLaunchArgument(
            'scenario',
            default_value='patrol',
            description='Scenario to execute: patrol, obstacle, intruder, family, gas_leak, or full'
        ),
        DeclareLaunchArgument(
            'gui_view',
            default_value='default',
            description='Gazebo GUI camera viewpoint: default (angled overview), topdown (overhead bird-eye), follow (tracks robot)'
        ),
        DeclareLaunchArgument(
            'log_data',
            default_value='true',
            description='Automatically record run telemetry to results/<scenario>_<timestamp>/'
        ),
        OpaqueFunction(function=launch_setup)
    ])
