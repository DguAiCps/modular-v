#!/usr/bin/env python3
"""
Turtlebot3 Burger + ZED 2i Mapping Launch File
Purpose: Create a map using RTAB-Map SLAM

Usage:
    ros2 launch turtlebot3_zed_nav turtlebot3_zed_mapping.launch.py
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
                    'depth_texture_conf': '100',
                    'depth_max_range': '10.0',
            }.items()
        ),
        
        # Note: ZED will publish TF: base_link -> zed_camera_link -> ... -> zed_left_camera_optical_frame
        
        # ========== 3. RTAB-Map SLAM (Mapping Mode) ==========
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'subscribe_odom_info': False,
                'approx_sync': True,
                'queue_size': 10,
                
                'database_path': os.path.expanduser('~/.ros/rtabmap_burger.db'),
                
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                
                'publish_tf': True,
                
                'RGBD/OptimizeMaxError': '1.0',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'Rtabmap/TimeThr': '0',
                'Rtabmap/DetectionRate': '1.0',
                'Mem/RehearsalSimilarity': '0.45',
                'Mem/NotLinkedNodesKept': 'true',
                'Mem/STMSize': '10',
                
                'Odom/Strategy': '0',
                'OdomF2M/MaxSize': '1000',
                'Vis/MinInliers': '15',
                'Vis/InlierDistance': '0.1',
            }],
            remappings=[
                ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
                ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
                ('depth/image', '/zed/zed_node/depth/depth_registered'),
            ],
            arguments=['--delete_db_on_start']
        ),
        
        # ========== 5. Map Server ==========
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
    ])
