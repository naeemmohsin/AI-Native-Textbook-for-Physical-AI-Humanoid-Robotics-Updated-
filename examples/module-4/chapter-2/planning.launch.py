#!/usr/bin/env python3
"""
Planning System Launch File

Launches the complete cognitive planning system:
1. LLM-based task planner
2. Action executor
3. Optional: Behavior tree executor

Usage:
    ros2 launch planning planning.launch.py

Arguments:
    model: LLM model to use (gpt-4, gpt-3.5-turbo, llama3.1:8b)
    use_local_llm: Whether to use local Ollama server
    simulate_actions: Whether to simulate action execution

Example:
    ros2 launch planning planning.launch.py \
        model:=gpt-4 \
        simulate_actions:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package paths
    pkg_share = FindPackageShare('planning')

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='gpt-4',
        description='LLM model to use for planning'
    )

    use_local_llm_arg = DeclareLaunchArgument(
        'use_local_llm',
        default_value='false',
        description='Use local Ollama server instead of OpenAI API'
    )

    local_llm_url_arg = DeclareLaunchArgument(
        'local_llm_url',
        default_value='http://localhost:11434',
        description='Local LLM server URL (for Ollama)'
    )

    temperature_arg = DeclareLaunchArgument(
        'temperature',
        default_value='0.2',
        description='LLM temperature (0.0-1.0)'
    )

    simulate_actions_arg = DeclareLaunchArgument(
        'simulate_actions',
        default_value='true',
        description='Simulate action execution (for testing)'
    )

    max_retries_arg = DeclareLaunchArgument(
        'max_retries',
        default_value='3',
        description='Maximum retries per action step'
    )

    step_timeout_arg = DeclareLaunchArgument(
        'step_timeout',
        default_value='60.0',
        description='Timeout for each action step (seconds)'
    )

    # Config file paths
    capabilities_config = PathJoinSubstitution([
        pkg_share, 'config', 'capabilities.yaml'
    ])

    prompt_template = PathJoinSubstitution([
        pkg_share, 'config', 'prompts', 'task_planner.txt'
    ])

    # LLM Planner Node
    planner_node = Node(
        package='llm_planner',
        executable='planner_node',
        name='llm_planner',
        parameters=[{
            'capabilities_file': capabilities_config,
            'model': LaunchConfiguration('model'),
            'temperature': LaunchConfiguration('temperature'),
            'max_tokens': 1500,
            'use_local_llm': LaunchConfiguration('use_local_llm'),
            'local_llm_url': LaunchConfiguration('local_llm_url'),
            'prompt_template_file': prompt_template,
        }],
        output='screen'
    )

    # Action Executor Node
    executor_node = Node(
        package='action_executor',
        executable='executor_node',
        name='action_executor',
        parameters=[{
            'max_retries': LaunchConfiguration('max_retries'),
            'step_timeout': LaunchConfiguration('step_timeout'),
            'abort_on_failure': True,
            'simulate_actions': LaunchConfiguration('simulate_actions'),
        }],
        output='screen'
    )

    # Optional: Status monitor node
    # monitor_node = Node(
    #     package='planning',
    #     executable='status_monitor',
    #     name='status_monitor',
    #     output='screen'
    # )

    return LaunchDescription([
        # Launch arguments
        model_arg,
        use_local_llm_arg,
        local_llm_url_arg,
        temperature_arg,
        simulate_actions_arg,
        max_retries_arg,
        step_timeout_arg,

        # Log configuration
        LogInfo(msg=['Launching cognitive planning system']),
        LogInfo(msg=['  Model: ', LaunchConfiguration('model')]),
        LogInfo(msg=['  Local LLM: ', LaunchConfiguration('use_local_llm')]),
        LogInfo(msg=['  Simulate actions: ', LaunchConfiguration('simulate_actions')]),

        # Nodes
        planner_node,
        executor_node,
        # monitor_node,
    ])
