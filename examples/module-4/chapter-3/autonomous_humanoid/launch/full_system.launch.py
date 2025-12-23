#!/usr/bin/env python3
"""
Full System Launch File for Autonomous Humanoid

This launch file starts the complete autonomous humanoid system including:
- Voice pipeline (audio capture, Whisper, intent parsing)
- LLM planner and action executor
- Navigation (Nav2)
- Perception (Isaac ROS)
- State machine orchestrator
- Monitoring dashboard

Usage:
    ros2 launch autonomous_humanoid full_system.launch.py
    ros2 launch autonomous_humanoid full_system.launch.py use_sim:=true
    ros2 launch autonomous_humanoid full_system.launch.py enable_voice:=true enable_planning:=true
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('autonomous_humanoid')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # Declare launch arguments
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation time'
    )

    declare_enable_voice = DeclareLaunchArgument(
        'enable_voice',
        default_value='true',
        description='Enable voice subsystem'
    )

    declare_enable_planning = DeclareLaunchArgument(
        'enable_planning',
        default_value='true',
        description='Enable LLM planning subsystem'
    )

    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav',
        default_value='true',
        description='Enable navigation (Nav2)'
    )

    declare_enable_perception = DeclareLaunchArgument(
        'enable_perception',
        default_value='true',
        description='Enable perception (Isaac ROS)'
    )

    declare_enable_dashboard = DeclareLaunchArgument(
        'enable_dashboard',
        default_value='true',
        description='Enable monitoring dashboard'
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'humanoid_config.yaml'),
        description='Path to system configuration file'
    )

    # Launch configurations
    use_sim = LaunchConfiguration('use_sim')
    enable_voice = LaunchConfiguration('enable_voice')
    enable_planning = LaunchConfiguration('enable_planning')
    enable_nav = LaunchConfiguration('enable_nav')
    enable_perception = LaunchConfiguration('enable_perception')
    enable_dashboard = LaunchConfiguration('enable_dashboard')
    config_file = LaunchConfiguration('config_file')

    # Set environment variables
    set_use_sim_time = SetEnvironmentVariable(
        name='USE_SIM_TIME',
        value=use_sim
    )

    # =========================================
    # Voice Subsystem
    # =========================================
    voice_group = GroupAction(
        condition=IfCondition(enable_voice),
        actions=[
            PushRosNamespace('voice'),

            # Audio capture node
            Node(
                package='audio_common',
                executable='audio_capture_node',
                name='audio_capture',
                parameters=[{
                    'device': 'default',
                    'sample_rate': 16000,
                    'channels': 1,
                    'use_sim_time': use_sim,
                }],
                remappings=[
                    ('audio', '/voice/audio'),
                ],
                output='screen'
            ),

            # Whisper STT node
            Node(
                package='whisper_ros',
                executable='whisper_node',
                name='whisper_stt',
                parameters=[{
                    'model': 'base',
                    'device': 'cuda',
                    'language': 'en',
                    'use_sim_time': use_sim,
                }],
                remappings=[
                    ('audio_in', '/voice/audio'),
                    ('transcription', '/voice/transcription'),
                ],
                output='screen'
            ),

            # Intent parser node
            Node(
                package='voice_control',
                executable='intent_parser_node',
                name='intent_parser',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim},
                ],
                remappings=[
                    ('transcription', '/voice/transcription'),
                    ('intent', '/voice/intent'),
                ],
                output='screen'
            ),

            # Wake word detector (optional)
            Node(
                package='voice_control',
                executable='wake_word_node',
                name='wake_word',
                parameters=[{
                    'engine': 'porcupine',
                    'keywords': ['hey robot'],
                    'sensitivity': 0.5,
                    'use_sim_time': use_sim,
                }],
                output='screen'
            ),
        ]
    )

    # =========================================
    # Planning Subsystem
    # =========================================
    planning_group = GroupAction(
        condition=IfCondition(enable_planning),
        actions=[
            PushRosNamespace('planning'),

            # LLM planner node
            Node(
                package='llm_planner',
                executable='planner_node',
                name='llm_planner',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim},
                ],
                remappings=[
                    ('intent', '/voice/intent'),
                    ('plan', '/planning/plan'),
                ],
                output='screen'
            ),

            # Action executor node
            Node(
                package='action_executor',
                executable='executor_node',
                name='action_executor',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim},
                ],
                remappings=[
                    ('plan', '/planning/plan'),
                    ('progress', '/executor/progress'),
                ],
                output='screen'
            ),
        ]
    )

    # =========================================
    # Navigation Subsystem (Nav2)
    # =========================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        condition=IfCondition(enable_nav),
        launch_arguments={
            'use_sim_time': use_sim,
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        }.items()
    )

    # =========================================
    # Perception Subsystem (Isaac ROS)
    # =========================================
    perception_group = GroupAction(
        condition=IfCondition(enable_perception),
        actions=[
            PushRosNamespace('perception'),

            # Isaac ROS object detection
            Node(
                package='isaac_ros_detectnet',
                executable='detectnet_node',
                name='object_detection',
                parameters=[{
                    'confidence_threshold': 0.7,
                    'use_sim_time': use_sim,
                }],
                remappings=[
                    ('image', '/camera/rgb/image_raw'),
                    ('detections', '/perception/detections'),
                ],
                output='screen'
            ),

            # Isaac ROS Visual SLAM
            Node(
                package='isaac_ros_visual_slam',
                executable='visual_slam_node',
                name='visual_slam',
                parameters=[{
                    'publish_tf': True,
                    'use_sim_time': use_sim,
                }],
                remappings=[
                    ('image', '/camera/rgb/image_raw'),
                    ('camera_info', '/camera/rgb/camera_info'),
                ],
                output='screen'
            ),

            # Scene graph maintainer
            Node(
                package='perception',
                executable='scene_graph_node',
                name='scene_graph',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim},
                ],
                output='screen'
            ),
        ]
    )

    # =========================================
    # State Machine Orchestrator
    # =========================================
    state_machine_node = Node(
        package='autonomous_humanoid',
        executable='state_machine_node',
        name='state_machine',
        parameters=[
            config_file,
            {'use_sim_time': use_sim},
        ],
        output='screen'
    )

    # Delay state machine start to allow other nodes to initialize
    delayed_state_machine = TimerAction(
        period=5.0,  # Wait 5 seconds
        actions=[state_machine_node]
    )

    # =========================================
    # Monitoring and Dashboard
    # =========================================
    monitoring_group = GroupAction(
        condition=IfCondition(enable_dashboard),
        actions=[
            # ROS 2 Web Bridge
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge',
                parameters=[{
                    'port': 9090,
                    'address': '0.0.0.0',
                    'use_sim_time': use_sim,
                }],
                output='screen'
            ),

            # YASMIN state machine viewer
            Node(
                package='yasmin_viewer',
                executable='yasmin_viewer_node',
                name='state_viewer',
                parameters=[{
                    'use_sim_time': use_sim,
                }],
                output='screen'
            ),

            # System logger
            Node(
                package='autonomous_humanoid',
                executable='system_logger_node',
                name='system_logger',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim},
                ],
                output='screen'
            ),
        ]
    )

    # =========================================
    # TF and Robot State
    # =========================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': '',  # Load from URDF file
            'use_sim_time': use_sim,
        }],
        output='screen'
    )

    # =========================================
    # Build Launch Description
    # =========================================
    return LaunchDescription([
        # Declare arguments
        declare_use_sim,
        declare_enable_voice,
        declare_enable_planning,
        declare_enable_nav,
        declare_enable_perception,
        declare_enable_dashboard,
        declare_config_file,

        # Environment
        set_use_sim_time,

        # Robot state
        robot_state_publisher,

        # Subsystems
        voice_group,
        planning_group,
        nav2_launch,
        perception_group,

        # Monitoring
        monitoring_group,

        # State machine (delayed start)
        delayed_state_machine,
    ])
