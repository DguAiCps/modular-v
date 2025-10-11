#!/usr/bin/env python3
"""
Turtlebot3 Burger + ZED 2i Navigation Launch File
Purpose: Use saved map for localization and autonomous navigation

Usage:
    ros2 launch turtlebot3_zed_nav turtlebot3_zed_navigation.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    return LaunchDescription([
        
        # ========== 1. Turtlebot3 Robot ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_bringup'),
                    'launch',
                    'robot.launch.py'
                ])
            ])
        ),
        
        # ========== 2. ZED 2i Camera ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('zed_wrapper'),
                    'launch',
                    'zed_camera.launch.py'
                ])
            ]),
            launch_arguments={
                'camera_model': 'zed2i',
                'camera_name': 'zed',
                'base_frame': 'base_link',
                'publish_tf': 'true',
                'publish_map_tf': 'false',
                'publish_urdf': 'true',
                'grab_resolution': 'HD1080',
                'grab_frame_rate': '10',
                'depth_mode': 'ULTRA',
                'depth_confidence': '80',
                'depth_max_range': '10.0',
            }.items()
        ),
        
        # ========== 3. RTAB-Map SLAM (Localization Mode) ==========
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                # Frame IDs
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                
                # Subscribe topics
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'subscribe_odom_info': False,
                'approx_sync': True,
                'queue_size': 10,
                
                # Database
                'database_path': os.path.expanduser('~/.ros/rtabmap_burger.db'),
                
                # LOCALIZATION MODE
                'Mem/IncrementalMemory': 'false',  # Don't add new data
                'Mem/InitWMWithAllNodes': 'true',   # Load all previous nodes
                
                # TF Publishing - CRITICAL!
                'publish_tf': True,
                
                # Loop closure for localization
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/OptimizeMaxError': '1.0',
                'Rtabmap/StartNewMapOnLoopClosure': 'false',
                
                # Detection
                'Rtabmap/DetectionRate': '2.0',
                'Mem/RehearsalSimilarity': '0.30',
            }],
            remappings=[
                ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
                ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
                ('depth/image', '/zed/zed_node/depth/depth_registered'),
            ]
        ),
        
        # ========== 4. Map Server ==========
        Node(
            package='rtabmap_util',
            executable='map_assembler',
            name='map_assembler',
            output='screen',
            parameters=[{
                'frame_id': 'map',
            }],
            remappings=[
                ('grid_map', '/map'),
            ]
        ),
        
        # ========== 5. Nav2 Navigation Stack ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'false',
            }.items()
        ),
    ])
