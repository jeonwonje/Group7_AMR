#!/usr/bin/env python3
"""
Integrated Launch file for Camera Driver, Vision Pipeline, and AprilTag Dock Pose Publishing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

REDUCED_HEIGHT, REDUCED_WIDTH = 480, 640
FRAMERATE = 10.0

def generate_launch_description():
    # Constants for paths
    package_name_docking = 'apriltag_docking'
    apriltag_ros_config_filename = 'apriltags_36h11.yaml'
    
    # IMPORTANT: Rename your calibration file to match this, or pass it as a launch argument
    camera_calibration_filename = 'camera_calibration.yaml' 

    pkg_share_docking = FindPackageShare(package=package_name_docking).find(package_name_docking)
    default_apriltag_ros_config_file_path = PathJoinSubstitution(
        [pkg_share_docking, 'config', apriltag_ros_config_filename])

    # In ROS, camera_info_manager requires a specific URI format (package:// or file://)
    default_camera_info_url = f'package://{package_name_docking}/config/{camera_calibration_filename}'

    # Launch configuration variables
    apriltag_config_file = LaunchConfiguration('apriltag_config_file')
    camera_calibration_url = LaunchConfiguration('camera_calibration_url')
    camera_frame_type = LaunchConfiguration('camera_frame_type')
    camera_namespace = LaunchConfiguration('camera_namespace')
    tag_family = LaunchConfiguration('tag_family')
    tag_id = LaunchConfiguration('tag_id') 
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_apriltag_config_file_cmd = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=default_apriltag_ros_config_file_path,
        description='Full path to the AprilTag config file to use')

    declare_camera_calibration_cmd = DeclareLaunchArgument(
        name='camera_calibration_url',
        default_value=default_camera_info_url,
        description='URI path to the camera calibration YAML file (e.g., package://...)')

    declare_camera_frame_type_cmd = DeclareLaunchArgument(
        name='camera_frame_type',
        default_value='camera',
        description='Type of camera frame to use')

    declare_camera_namespace_cmd = DeclareLaunchArgument(
        name='camera_namespace',
        default_value='',
        description='Namespace for the camera and AprilTag nodes')

    declare_tag_family_cmd = DeclareLaunchArgument(
        name='tag_family',
        default_value='tag36h11',
        description='Family of AprilTag being used')

    declare_tag_id_cmd = DeclareLaunchArgument(
        name='tag_id',
        default_value='0',
        description='ID of the AprilTag being used')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true (false by default)')

    # ==========================================
    # VISION PIPELINE COMPOSABLE NODES
    # ==========================================

    # 1. Hardware Driver (Force Binned 1640x1232 for Full FOV)
    camera_node = ComposableNode(
        package='camera_ros',
        plugin='camera::CameraNode',
        name='camera_node',
        namespace=camera_namespace,
        # FORCE the node to output to the exact topics the resizer expects
        remappings=[
            ('~/image_raw', '/camera/image_raw'),
            ('~/camera_info', '/camera/camera_info'),
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        parameters=[{
            'width': 1640,
            'height': 1232,
            'format': 'BGR888', 
            'camera_info_url': camera_calibration_url,
            'use_sim_time': use_sim_time,
            'framerate': FRAMERATE
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 2. Resizer 
    resize_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::ResizeNode',
        name='resize_node',
        namespace=camera_namespace,
        remappings=[
            ('image/image_raw', '/camera/image_raw'),
            ('image/camera_info', '/camera/camera_info'),
            ('resize/image_raw', '/camera/resized/image_raw'),
            ('resize/camera_info', '/camera/resized/camera_info'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_scale': False, # <--- THE FATAL FLAW FIX
            'width': REDUCED_WIDTH,
            'height': REDUCED_HEIGHT,
            'interpolation': 0  # <--- Use Nearest Neighbor (Fastest)
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 3. Rectifier 
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_node',
        namespace=camera_namespace,
        remappings=[
            ('image', '/camera/resized/image_raw'),
            ('camera_info', '/camera/resized/camera_info'),
            ('image_rect', '/camera/resized/image_rect') 
        ],
        parameters=[{
            'queue_size': 5,
            'interpolation': 0, # <--- Use Nearest Neighbor (Fastest)
            'use_sim_time': use_sim_time,
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 4. AprilTag Detector
    apriltag_ros_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag_node',
        namespace=camera_namespace,
        remappings=[
            # FIX: Read the rectified image from the unified namespace
            ('image_rect', '/camera/resized/image_rect'), 
            # image_transport will now automatically find /camera/resized/camera_info
            ('camera_info', '/camera/resized/camera_info'),
        ],
        parameters=[
            apriltag_config_file,
            {'use_sim_time': use_sim_time}
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Combine all 4 into a single zero-copy multi-threaded process
    start_vision_pipeline_container = ComposableNodeContainer(
        name='apriltag_vision_container',
        namespace=camera_namespace,
        package='rclcpp_components',
        executable='component_container', 
        composable_node_descriptions=[
            camera_node,
            resize_node,
            rectify_node,
            apriltag_ros_node
        ],
        output='screen'
    )

    # ==========================================
    # PHYSICAL ROBOT TF BRIDGES (Retained)
    # ==========================================
    base_to_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=[
            '--x', '0.09', '--y', '0.05', '--z', '0.097', 
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',  
            '--frame-id', 'base_link', '--child-frame-id', 'camera_link'
        ]
    )

    camera_optical_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_bridge',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '-1.5708', '--pitch', '0', '--roll', '-1.5708',
            '--frame-id', 'camera_link', '--child-frame-id', 'camera'
        ]
    )

    # ==========================================
    # STATION 0 PIPELINE (ID: 0)
    # ==========================================
    dock_target_node_0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dock_target_broadcaster_0',
        arguments=[
            '--x', '0.195', '--y', '0', '--z', '0.05', 
            '--yaw', '0', '--pitch', '-1.5708', '--roll', '1.5708', 
            '--frame-id', [tag_family, ':', '0'], '--child-frame-id', 'nav2_dock_target_0' 
        ]
    )

    start_detected_dock_pose_publisher_0 = Node(
        package='apriltag_docking',
        executable='detected_dock_pose_publisher',
        name='detected_dock_pose_publisher_0',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': camera_frame_type,
            'child_frame': 'nav2_dock_target_0',  
            'publish_rate': 10.0
        }],
        remappings=[('detected_dock_pose', 'detected_dock_pose_0')],
        output='screen'
    )

    # ==========================================
    # STATION 2 PIPELINE (ID: 2)
    # ==========================================
    dock_target_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dock_target_broadcaster_2',
        arguments=[
            '--x', '0.195', '--y', '0', '--z', '0.05', 
            '--yaw', '0', '--pitch', '-1.5708', '--roll', '1.5708', 
            '--frame-id', [tag_family, ':', '2'], '--child-frame-id', 'nav2_dock_target_2' 
        ]
    )

    start_detected_dock_pose_publisher_2 = Node(
        package='apriltag_docking',
        executable='detected_dock_pose_publisher',
        name='detected_dock_pose_publisher_2',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': camera_frame_type,
            'child_frame': 'nav2_dock_target_2',  
            'publish_rate': 10.0
        }],
        remappings=[('detected_dock_pose', 'detected_dock_pose_2')],
        output='screen'
    )
    
    # ==========================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================
    ld = LaunchDescription()

    ld.add_action(declare_apriltag_config_file_cmd)
    ld.add_action(declare_camera_calibration_cmd)
    ld.add_action(declare_camera_frame_type_cmd)
    ld.add_action(declare_camera_namespace_cmd)
    ld.add_action(declare_tag_family_cmd)
    ld.add_action(declare_tag_id_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the single vision pipeline container
    ld.add_action(start_vision_pipeline_container)
    
    # Add TF Bridges
    ld.add_action(base_to_camera_node)
    ld.add_action(camera_optical_node)
    
    # Add Docking Pose Logic
    ld.add_action(dock_target_node_0)
    ld.add_action(start_detected_dock_pose_publisher_0)
    ld.add_action(dock_target_node_2)
    ld.add_action(start_detected_dock_pose_publisher_2)

    return ld
