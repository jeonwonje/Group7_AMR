#!/usr/bin/env python3
"""
Full warehouse mission launch for CDE2310 AMR.

Launches on the WSL2 side:
  1. SLAM Toolbox (online_sync mode)
  2. Static TF for camera frames
  3. auto_nav (custom navigation + delivery states)
  4. apriltag_detector
  5. coverage_monitor
  6. RViz2 (optional)

No Nav2 dependency.

RPi side (launch separately):
  ros2 launch turtlebot3_bringup robot.launch.py
  ros2 launch turtlebot3_bringup camera.launch.py

Usage:
  ros2 launch amr_nav mission.launch.py
  ros2 launch amr_nav mission.launch.py use_rviz:=false marker_size:=0.08
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    amr_nav_dir = get_package_share_directory('amr_nav')
    slam_params = os.path.join(amr_nav_dir, 'config', 'slam_params.yaml')
    nav_tuning = os.path.join(amr_nav_dir, 'config', 'nav_tuning.yaml')

    # ── Arguments ────────────────────────────────────────
    rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    marker_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.16',
        description='AprilTag marker side length in metres')

    # ── SLAM Toolbox ─────────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params])

    # ── Static TF: base_link → camera_link ───────────────
    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=[
            '--x', '0.07', '--y', '0', '--z', '0.15',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ])

    # camera_link → camera_optical_frame
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_optical_frame',
        ])

    # ── Custom navigation ────────────────────────────────
    auto_nav_node = Node(
        package='amr_nav',
        executable='auto_nav',
        name='auto_nav',
        output='screen',
        parameters=[nav_tuning])

    # ── AprilTag detector ────────────────────────────────
    apriltag_node = Node(
        package='amr_perception',
        executable='apriltag_detector',
        name='apriltag_detector',
        output='screen',
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'detection_rate': 10.0,
        }])

    # ── Coverage monitor ─────────────────────────────────
    coverage_node = Node(
        package='amr_nav',
        executable='coverage_monitor',
        name='coverage_monitor',
        output='screen')

    # ── RViz2 ────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        rviz_arg, marker_arg,
        slam_node,
        camera_link_tf, camera_optical_tf,
        auto_nav_node, apriltag_node, coverage_node,
        rviz_node,
    ])
