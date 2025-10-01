from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Package paths
    pkg_modular_v = FindPackageShare('modular_v')
    pkg_zed_wrapper = FindPackageShare('zed_wrapper')
    pkg_rtabmap_launch = FindPackageShare('rtabmap_launch')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_slam = LaunchConfiguration('enable_slam', default='true')
    enable_navigation = LaunchConfiguration('enable_navigation', default='true')
    enable_voice = LaunchConfiguration('enable_voice', default='true')
    config_dir = LaunchConfiguration('config_dir',
        default=PathJoinSubstitution([pkg_modular_v, 'config']))

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')

    declare_enable_slam = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable RTAB-Map SLAM')

    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Enable Nav2 navigation')

    declare_enable_voice = DeclareLaunchArgument(
        'enable_voice',
        default_value='true',
        description='Enable voice interface')

    declare_config_dir = DeclareLaunchArgument(
        'config_dir',
        default_value=PathJoinSubstitution([pkg_modular_v, 'config']),
        description='Configuration directory')

    # ZED Camera launch
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_zed_wrapper, 'launch', 'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_tf': 'true',
            'publish_map_tf': 'false',
            'depth_mode': 'ULTRA',
            'depth_stabilization': '1',
            'pos_tracking_enabled': 'true',
            'base_frame': 'base_link',
            'cam_frame': 'zed2i_base_link',
            'publish_urdf': 'true',
        }.items()
    )

    # RTAB-Map launch
    rtabmap_launch = GroupAction(
        condition=IfCondition(enable_slam),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_rtabmap_launch, 'launch', 'rtabmap.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'args': '--delete_db_on_start',
                    'rgb_topic': '/zed2i/zed_node/rgb/image_rect_color',
                    'depth_topic': '/zed2i/zed_node/depth/depth_registered',
                    'camera_info_topic': '/zed2i/zed_node/rgb/camera_info',
                    'frame_id': 'base_link',
                    'approx_sync': 'false',
                    'wait_imu_to_init': 'true',
                    'imu_topic': '/zed2i/zed_node/imu/data',
                    'rviz': 'false'
                }.items()
            )
        ]
    )

    # Navigation2 launch
    nav2_launch = GroupAction(
        condition=IfCondition(enable_navigation),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_nav2_bringup, 'launch', 'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': PathJoinSubstitution([
                        config_dir, 'navigation_params.yaml'
                    ]),
                    'map_subscribe_transient_local': 'true'
                }.items()
            )
        ]
    )

    # Module Manager Node
    module_manager_node = Node(
        package='modular_v',
        executable='modular_v_main',
        name='module_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': PathJoinSubstitution([
                config_dir, 'system_config.yaml'
            ])
        }]
    )

    # Motor Controller Node
    motor_controller_node = Node(
        package='modular_v',
        executable='motor_controller_node',
        name='motor_controller',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                config_dir, 'motor_config.yaml'
            ])
        ]
    )

    # Voice Interface Node
    voice_interface_node = GroupAction(
        condition=IfCondition(enable_voice),
        actions=[
            Node(
                package='modular_v',
                executable='voice_interface_node',
                name='voice_interface',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        config_dir, 'voice_config.yaml'
                    ])
                ]
            )
        ]
    )

    # System Monitor Node
    system_monitor_node = Node(
        package='modular_v',
        executable='system_health_check.py',
        name='system_monitor',
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_enable_slam,
        declare_enable_navigation,
        declare_enable_voice,
        declare_config_dir,

        # Launch nodes and includes
        zed_camera_launch,
        rtabmap_launch,
        nav2_launch,
        module_manager_node,
        motor_controller_node,
        voice_interface_node,
        system_monitor_node
    ])