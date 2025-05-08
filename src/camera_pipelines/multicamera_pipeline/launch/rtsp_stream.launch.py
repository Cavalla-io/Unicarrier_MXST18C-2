#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Define launch arguments
    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://admin:admin@192.168.2.250:554/stream',
        description='URL of the RTSP stream'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='camera/image_raw',
        description='Name of the topic to publish images to'
    )
    
    pipeline_type_arg = DeclareLaunchArgument(
        'pipeline_type',
        default_value='4',  # Use TCP pipeline by default
        description='Pipeline type (1-4): 1=UDP, 2=TCP, 3=UDP with queue, 4=TCP with queue'
    )
    
    # Create node
    rtsp_node = Node(
        package='multicamera_pipeline',
        executable='rtsp_stream_node',
        name='rtsp_stream_node',
        parameters=[{
            'rtsp_url': LaunchConfiguration('rtsp_url'),
            'topic_name': LaunchConfiguration('topic_name'),
            'pipeline_type': LaunchConfiguration('pipeline_type'),
            'frame_rate': 20.0,  # Update to match the actual stream frame rate
            'image_width': 800,  # Match the actual stream resolution
            'image_height': 448,  # Match the actual stream resolution
        }],
        output='screen',
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(rtsp_url_arg)
    ld.add_action(topic_name_arg)
    ld.add_action(pipeline_type_arg)
    
    # Add node to launch description
    ld.add_action(rtsp_node)
    
    return ld 