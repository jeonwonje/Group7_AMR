#!/usr/bin/env python3
"""
full_mission.launch.py — Single launch file for the complete remote PC mission.

Replaces these 4 terminal commands:
  1. ros2 launch CDE2310_AMR_Trial_Run minimal_nav2.launch.py use_sim_time:=false
  2. ros2 run rviz2 rviz2 -d .../nav2_default_view.rviz
  3. ros2 launch CDE2310_AMR_Trial_Run mission.launch.py use_sim_time:=false
  4. ros2 launch auto_explore_v2 auto_explore.launch.py use_sim_time:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    pkg_dir = get_package_share_directory('CDE2310_AMR_Trial_Run')
    nav2_yaml = os.path.join(pkg_dir, 'config', 'minimal_nav2.yaml')
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    
    # RViz Config Path
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
        
    # Mission and Exploration Launch Paths
    mission_launch_path = os.path.join(
        pkg_dir, 'launch', 'mission.launch.py')
        
    auto_explore_launch_path = os.path.join(
        get_package_share_directory('auto_explore_v2'), 'launch', 'auto_explore.launch.py')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    sim_time_param = {'use_sim_time': use_sim_time}

    # ==================================================================
    # 1. SLAM — Cartographer
    # ==================================================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[sim_time_param],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer.lua',
            '--ros-args', '--log-level', 'WARN',
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            sim_time_param,
            {'resolution': 0.025}, # Updated to match your high-res tight maze config
            {'publish_period_sec': 1.0},
        ],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    # ==================================================================
    # 2. Nav2 stack
    # ==================================================================
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['planner_server',
                            'controller_server',
                            'behavior_server',
                            'bt_navigator']}
        ]
    )

    # ==================================================================
    # 3. Visualization
    # ==================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[sim_time_param],
        output='screen'
    )

    # ==================================================================
    # 4. Mission & Exploration (DELAYED LAUNCH)
    # ==================================================================
    delayed_mission_launch = TimerAction(
        period=15.0,  # Wait 15 seconds for SLAM and Nav2 to stabilize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_launch_path),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(auto_explore_launch_path),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # ==================================================================
    # Build Launch Description
    # ==================================================================
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time)
    
    # SLAM
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    
    # Nav2
    ld.add_action(planner_server)
    ld.add_action(controller_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager_navigation)
    
    # UI
    ld.add_action(rviz_node)
    
    # Delayed High-Level Logic
    ld.add_action(delayed_mission_launch)

    return ld
