#!/usr/bin/env python3
"""
slam_test.launch.py — Minimal SLAM Toolbox test in Gazebo.

Starts ONLY: Gazebo + TurtleBot3 burger + SLAM Toolbox + RViz.
Drive with teleop in a separate terminal.

Usage:
  export TURTLEBOT3_MODEL=burger
  ros2 launch CDE2310_AMR_Trial_Run slam_test.launch.py

  # Separate terminal:
  export TURTLEBOT3_MODEL=burger
  ros2 run turtlebot3_teleop teleop_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('CDE2310_AMR_Trial_Run')
    slam_yaml = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_sdf = os.path.join(
        tb3_gazebo_dir, 'models',
        'turtlebot3_' + tb3_model, 'model.sdf')

    # World: use our maze, fall back to stock world
    amr_maze = os.path.join(pkg_dir, 'worlds', 'amr_maze.world')
    world_file = amr_maze if os.path.isfile(amr_maze) else os.path.join(
        tb3_gazebo_dir, 'worlds', 'turtlebot3_world.world')

    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_time_param = {'use_sim_time': use_sim_time}

    # ---- Gazebo ----
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    # ---- Robot State Publisher (URDF TFs) ----
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch',
                         'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # ---- Spawn robot (delayed — give gzserver time to load) ----
    spawn_robot = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'burger',
                    '-file', model_sdf,
                    '-x', '0.4',
                    '-y', '0.4',
                    '-z', '0.01',
                    '-timeout', '120',
                ],
                output='screen',
            ),
        ],
    )

    # ---- SLAM Toolbox (delayed — wait for robot spawn + first scans) ----
    slam_toolbox = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_yaml, sim_time_param],
            ),
        ],
    )

    # ---- RViz ----
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[sim_time_param],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'),

        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        slam_toolbox,
        rviz,
    ])
