#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Set environment variables to suppress warnings
    gst_debug = SetEnvironmentVariable(
        name='GST_DEBUG',
        value='0'
    )
    
    gst_silent = SetEnvironmentVariable(
        name='GST_SILENT',
        value='1'
    )
    
    opencv_log = SetEnvironmentVariable(
        name='OPENCV_LOG_LEVEL',
        value='0'
    )
    
    ros_log = SetEnvironmentVariable(
        name='RCUTILS_LOGGING_LEVEL',
        value='FATAL'
    )
    
    # Define launch arguments
    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://192.168.2.250:554/stream',
        description='URL of the RTSP stream'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='rtsp_camera1/image_raw',
        description='Name of the topic to publish images to'
    )
    
    pipeline_type_arg = DeclareLaunchArgument(
        'pipeline_type',
        default_value='4',  # Use TCP pipeline by default
        description='Pipeline type (1-4): 1=UDP, 2=TCP, 3=UDP with queue, 4=TCP with queue'
    )
    
    silent_mode_arg = DeclareLaunchArgument(
        'silent_mode',
        default_value='true',
        description='Run in silent mode with minimal terminal output'
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
            'frame_rate': 15.0,  # Reduced frame rate for reliability
            'image_width': 640,  # Match actual stream resolution
            'image_height': 480,  # Match actual stream resolution
            'silent_mode': LaunchConfiguration('silent_mode'),
            'connection_retry_interval': 5,  # Wait 5 seconds between retries
            'max_connection_attempts': 20,  # Maximum retry attempts
            'use_sim_time': False
        }],
        output='log',  # Redirect output to log files
        remappings=[
            # Disable all possible image transport plugins
            ('/rtsp_camera1/image_raw/compressed', '/rtsp_camera1/disabled_compressed'),
            ('/rtsp_camera1/image_raw/compressedDepth', '/rtsp_camera1/disabled_compressedDepth'),
            ('/rtsp_camera1/image_raw/theora', '/rtsp_camera1/disabled_theora'),
        ],
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add environment variables first
    ld.add_action(gst_debug)
    ld.add_action(gst_silent)
    ld.add_action(opencv_log)
    ld.add_action(ros_log)
    
    # Add launch arguments
    ld.add_action(rtsp_url_arg)
    ld.add_action(topic_name_arg)
    ld.add_action(pipeline_type_arg)
    ld.add_action(silent_mode_arg)
    
    # Add node to launch description
    ld.add_action(rtsp_node)
    
    return ld 