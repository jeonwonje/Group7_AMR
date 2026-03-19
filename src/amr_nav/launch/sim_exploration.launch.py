#!/usr/bin/env python3
"""
Gazebo simulation launch for custom navigation exploration.

Launches:
  1. gzserver (headless — no gzclient by default for WSL2)
  2. robot_state_publisher + spawn TurtleBot3
  3. SLAM Toolbox (delayed 10 s)
  4. auto_nav + coverage_monitor (delayed 12 s)
  5. RViz2 (optional, delayed 10 s)

No Nav2 dependency.

Usage:
  ros2 launch amr_nav sim_exploration.launch.py
  ros2 launch amr_nav sim_exploration.launch.py use_rviz:=false
  ros2 launch amr_nav sim_exploration.launch.py coverage_threshold:=0.90
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    amr_nav_dir = get_package_share_directory('amr_nav')
    slam_params_sim = os.path.join(
        amr_nav_dir, 'config', 'slam_params_sim.yaml')
    nav_tuning = os.path.join(amr_nav_dir, 'config', 'nav_tuning.yaml')

    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ── Launch arguments ─────────────────────────────────
    x_arg = DeclareLaunchArgument('x_pose', default_value='-2.0')
    y_arg = DeclareLaunchArgument('y_pose', default_value='-0.5')
    cov_arg = DeclareLaunchArgument(
        'coverage_threshold', default_value='0.95')
    rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    gui_arg = DeclareLaunchArgument('use_gui', default_value='false')

    # ── 1. Gazebo (headless by default) ──────────────────
    world = os.path.join(
        gazebo_dir, 'worlds', 'turtlebot3_world.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_gui')))

    # ── 2. Robot ─────────────────────────────────────────
    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch',
                         'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch',
                         'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items())

    # ── 3. SLAM Toolbox ──────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_sim, {'use_sim_time': True}])

    # ── 4. Custom navigation ─────────────────────────────
    auto_nav_node = Node(
        package='amr_nav',
        executable='auto_nav',
        name='auto_nav',
        output='screen',
        parameters=[nav_tuning, {'use_sim_time': True}])

    coverage_node = Node(
        package='amr_nav',
        executable='coverage_monitor',
        name='coverage_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'coverage_threshold': LaunchConfiguration('coverage_threshold'),
        }])

    # ── 5. RViz2 ─────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        x_arg, y_arg, cov_arg, rviz_arg, gui_arg,
        # Gazebo (immediate)
        gzserver, gzclient, robot_state_pub, spawn_robot,
        # SLAM (delayed 10 s for Gazebo init)
        TimerAction(period=10.0, actions=[slam_node]),
        # Navigation + coverage (delayed 12 s for SLAM init)
        TimerAction(period=12.0, actions=[auto_nav_node, coverage_node]),
        # RViz (delayed 10 s)
        TimerAction(period=10.0, actions=[rviz_node]),
    ])
