#!/usr/bin/env python3
"""
nav_tuner.launch.py — Cartographer + Nav2 + Exploration + RViz in one terminal.
No mission nodes (search, docking, delivery, coordinator).
Use this to tune navigation and frontier exploration behaviour.

Usage:
  ros2 launch CDE2310_AMR_Trial_Run nav_tuner.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('CDE2310_AMR_Trial_Run')
    nav2_yaml = os.path.join(pkg_dir, 'config', 'minimal_nav2.yaml')
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    sim_time_param = {'use_sim_time': use_sim_time}

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        declare_use_sim_time,

        # 1. Cartographer SLAM
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='log',
            parameters=[sim_time_param],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'cartographer.lua',
                '--ros-args', '--log-level', 'WARN',
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='log',
            parameters=[
                sim_time_param,
                {'resolution': 0.05},
                {'publish_period_sec': 1.0},
            ],
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),

        # 2. Nav2 stack
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml, sim_time_param],
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml, sim_time_param],
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_yaml, sim_time_param],
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, sim_time_param],
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                sim_time_param,
                {'autostart': True},
                {'node_names': lifecycle_nodes},
                {'bond_timeout': 8.0},
            ],
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),

        # 3. Frontier exploration
        Node(
            package='auto_explore_v2',
            executable='find_frontiers',
            name='auto_explore',
            output='screen',
            parameters=[sim_time_param],
        ),
        Node(
            package='auto_explore_v2',
            executable='score_and_post',
            name='score_and_post',
            output='screen',
            parameters=[sim_time_param],
        ),

        # 4. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config, '--ros-args', '--log-level', 'WARN'],
        ),
    ])
