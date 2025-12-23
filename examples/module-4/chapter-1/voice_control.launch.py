#!/usr/bin/env python3
"""
Voice Control Launch File

Launches the complete voice control pipeline:
1. Audio capture from microphone
2. Whisper speech-to-text
3. Intent parsing
4. Action routing (optional)

Usage:
    ros2 launch voice_control voice_control.launch.py

Arguments:
    model_size: Whisper model size (tiny, base, small, medium, large-v3)
    language: Language code (en, es, fr, etc.)
    enable_wake_word: Whether to use wake-word detection
    audio_device: ALSA audio device name

Example:
    ros2 launch voice_control voice_control.launch.py \
        model_size:=small language:=en enable_wake_word:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package paths
    pkg_share = FindPackageShare('voice_control')

    # Declare launch arguments
    model_size_arg = DeclareLaunchArgument(
        'model_size',
        default_value='base',
        description='Whisper model size (tiny, base, small, medium, large-v3)',
        choices=['tiny', 'base', 'small', 'medium', 'large-v3']
    )

    language_arg = DeclareLaunchArgument(
        'language',
        default_value='en',
        description='Language code for speech recognition'
    )

    audio_device_arg = DeclareLaunchArgument(
        'audio_device',
        default_value='default',
        description='ALSA audio device name'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='16000',
        description='Audio sample rate in Hz'
    )

    enable_wake_word_arg = DeclareLaunchArgument(
        'enable_wake_word',
        default_value='false',
        description='Enable wake-word detection'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.7',
        description='Minimum confidence for intent recognition'
    )

    use_faster_whisper_arg = DeclareLaunchArgument(
        'use_faster_whisper',
        default_value='false',
        description='Use faster-whisper instead of openai-whisper'
    )

    # Config file paths
    audio_config = PathJoinSubstitution([
        pkg_share, 'config', 'audio_config.yaml'
    ])

    commands_config = PathJoinSubstitution([
        pkg_share, 'config', 'commands.yaml'
    ])

    # Audio capture node
    audio_capture_node = Node(
        package='audio_capture',
        executable='audio_capture_node',
        name='audio_capture',
        parameters=[{
            'device': LaunchConfiguration('audio_device'),
            'format': 'wave',
            'channels': 1,
            'sample_rate': LaunchConfiguration('sample_rate'),
            'sample_format': 'S16LE',
            'chunk_size': 512,
        }],
        remappings=[
            ('audio', '/voice/audio_raw'),
        ],
        output='screen'
    )

    # Whisper speech-to-text node
    whisper_node = Node(
        package='whisper_node',
        executable='whisper_node',
        name='whisper_stt',
        parameters=[{
            'model_size': LaunchConfiguration('model_size'),
            'language': LaunchConfiguration('language'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'buffer_seconds': 5.0,
            'silence_threshold': 0.01,
            'min_audio_length': 0.5,
            'use_faster_whisper': LaunchConfiguration('use_faster_whisper'),
            'device': 'cpu',  # Change to 'cuda' for GPU
        }],
        output='screen'
    )

    # Intent parser node
    intent_parser_node = Node(
        package='command_parser',
        executable='intent_parser',
        name='intent_parser',
        parameters=[{
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'commands_file': commands_config,
            'enable_llm_fallback': False,
            'debug_mode': False,
        }],
        output='screen'
    )

    # Optional wake-word detector
    # wake_word_node = Node(
    #     package='wake_word',
    #     executable='wake_word_detector',
    #     name='wake_word_detector',
    #     condition=IfCondition(LaunchConfiguration('enable_wake_word')),
    #     parameters=[{
    #         'model': 'hey_robot',
    #         'threshold': 0.5,
    #         'listen_timeout': 10.0,
    #     }],
    #     output='screen'
    # )

    return LaunchDescription([
        # Launch arguments
        model_size_arg,
        language_arg,
        audio_device_arg,
        sample_rate_arg,
        enable_wake_word_arg,
        confidence_threshold_arg,
        use_faster_whisper_arg,

        # Log configuration
        LogInfo(msg=['Launching voice control pipeline']),
        LogInfo(msg=['  Model: ', LaunchConfiguration('model_size')]),
        LogInfo(msg=['  Language: ', LaunchConfiguration('language')]),
        LogInfo(msg=['  Audio device: ', LaunchConfiguration('audio_device')]),

        # Nodes
        audio_capture_node,
        whisper_node,
        intent_parser_node,
        # wake_word_node,
    ])
