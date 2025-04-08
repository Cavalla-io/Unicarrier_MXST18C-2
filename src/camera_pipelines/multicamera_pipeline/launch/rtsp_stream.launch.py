#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Define launch arguments - only keep essential ones
    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://192.168.2.250:554/stream',
        description='URL of the RTSP stream'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Frame rate for capturing and publishing (Hz)'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Width of the image in pixels'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Height of the image in pixels'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='camera/image_raw',
        description='Name of the topic to publish images to'
    )
    
    pipeline_type_arg = DeclareLaunchArgument(
        'pipeline_type',
        default_value='1',
        description='Pipeline type (1-4): 1=UDP ultra-low latency, 2=Alternative optimized, 3=Hardware acceleration (if available), 4=Direct TCP'
    )
    
    # Create node
    rtsp_node = Node(
        package='multicamera_pipeline',
        executable='rtsp_stream_node',
        name='rtsp_stream_node',
        parameters=[{
            'rtsp_url': LaunchConfiguration('rtsp_url'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'topic_name': LaunchConfiguration('topic_name'),
            'pipeline_type': LaunchConfiguration('pipeline_type'),
        }],
        output='screen',
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(rtsp_url_arg)
    ld.add_action(frame_rate_arg)
    ld.add_action(image_width_arg)
    ld.add_action(image_height_arg)
    ld.add_action(topic_name_arg)
    ld.add_action(pipeline_type_arg)
    
    # Add nodes to launch description
    ld.add_action(rtsp_node)
    
    return ld 