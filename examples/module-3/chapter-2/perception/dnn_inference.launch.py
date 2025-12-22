#!/usr/bin/env python3
"""
Isaac ROS DNN Inference Launch File

This launch file configures and starts a TensorRT-accelerated deep learning
inference pipeline for object detection.

Usage:
    ros2 launch dnn_inference dnn_inference.launch.py \
        model_file_path:=/path/to/model.onnx

Requirements:
    - Isaac ROS DNN Inference package installed
    - ONNX or TensorRT engine model file
    - Camera providing RGB images
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for DNN inference pipeline."""

    # Declare launch arguments
    model_file_path_arg = DeclareLaunchArgument(
        'model_file_path',
        default_value='/workspaces/isaac_ros-dev/models/ssd_mobilenet_v2.onnx',
        description='Path to ONNX or TensorRT engine model file'
    )

    engine_file_path_arg = DeclareLaunchArgument(
        'engine_file_path',
        default_value='/tmp/ssd_mobilenet_v2.engine',
        description='Path to save/load TensorRT engine'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum confidence threshold for detections'
    )

    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/camera/left/image_resized',
        description='Input image topic'
    )

    # Get launch configurations
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    input_image_topic = LaunchConfiguration('input_image_topic')

    # DNN Image Encoder - Preprocesses images for the neural network
    dnn_encoder_node = Node(
        package='isaac_ros_dnn_inference',
        executable='dnn_image_encoder_node',
        name='dnn_encoder',
        output='screen',
        parameters=[{
            # Network input dimensions (SSD MobileNet expects 300x300)
            'network_image_width': 300,
            'network_image_height': 300,

            # Normalization parameters
            # For MobileNet: mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5]
            'image_mean': [0.5, 0.5, 0.5],
            'image_stddev': [0.5, 0.5, 0.5],

            # Input tensor configuration
            'num_blocks': 40,  # Number of memory blocks for NITROS
        }],
        remappings=[
            ('image', input_image_topic),
            ('encoded_tensor', '/tensor_pub'),
        ]
    )

    # TensorRT Inference Node - Runs the neural network
    tensor_rt_node = Node(
        package='isaac_ros_tensor_rt',
        executable='tensor_rt_node',
        name='tensor_rt',
        output='screen',
        parameters=[{
            # Model configuration
            'model_file_path': model_file_path,
            'engine_file_path': engine_file_path,

            # Tensor bindings (model-specific)
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['input'],
            'output_tensor_names': ['output_tensor'],
            'output_binding_names': ['detection_out', 'keep_count'],

            # Engine configuration
            'force_engine_update': False,  # Set True to rebuild engine
            'verbose': False,

            # Performance settings
            'max_workspace_size': 67108864,  # 64MB workspace
            'dla_core': -1,  # -1 for GPU, 0/1 for Jetson DLA cores
        }],
        remappings=[
            ('tensor_pub', '/tensor_pub'),
            ('tensor_sub', '/tensor_output'),
        ]
    )

    # Detection Decoder - Converts tensors to detection messages
    detection_decoder_node = Node(
        package='isaac_ros_dnn_inference',
        executable='detection2_d_array_decoder_node',
        name='detection_decoder',
        output='screen',
        parameters=[{
            # Detection parameters
            'confidence_threshold': confidence_threshold,
            'nms_threshold': 0.45,  # Non-maximum suppression threshold

            # Label configuration
            'label_list': [
                'person', 'bicycle', 'car', 'motorcycle', 'airplane',
                'bus', 'train', 'truck', 'boat', 'traffic light',
                'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
                'cat', 'dog', 'horse', 'sheep', 'cow',
                'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
                'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
                'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
                'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle',
                'wine glass', 'cup', 'fork', 'knife', 'spoon',
                'bowl', 'banana', 'apple', 'sandwich', 'orange',
                'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
                'cake', 'chair', 'couch', 'potted plant', 'bed',
                'dining table', 'toilet', 'tv', 'laptop', 'mouse',
                'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
                'toaster', 'sink', 'refrigerator', 'book', 'clock',
                'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush',
            ],  # COCO 80 classes
        }],
        remappings=[
            ('tensor_sub', '/tensor_output'),
            ('detection2_d_array', '/detections'),
        ]
    )

    # Detection Visualization - Publishes visualization markers
    detection_viz_node = Node(
        package='isaac_ros_dnn_inference',
        executable='detection_visualizer_node',
        name='detection_visualizer',
        output='screen',
        parameters=[{
            'rect_color': [0, 255, 0],  # Green bounding boxes
            'rect_thickness': 2,
            'text_color': [255, 255, 255],  # White text
            'text_scale': 0.6,
        }],
        remappings=[
            ('image', input_image_topic),
            ('detection2_d_array', '/detections'),
            ('detections_viz', '/detections_visualization'),
        ]
    )

    return LaunchDescription([
        model_file_path_arg,
        engine_file_path_arg,
        confidence_threshold_arg,
        input_image_topic_arg,
        dnn_encoder_node,
        tensor_rt_node,
        detection_decoder_node,
        detection_viz_node,
    ])
