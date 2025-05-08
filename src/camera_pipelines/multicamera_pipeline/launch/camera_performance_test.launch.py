import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Get package directories
    multicamera_prefix = get_package_share_directory("multicamera_pipeline")
    
    # Create configurable parameters
    display_debug_image = LaunchConfiguration('display_debug_image')
    log_to_file = LaunchConfiguration('log_to_file')
    monitor_period = LaunchConfiguration('monitor_period')
    run_duration = LaunchConfiguration('run_duration')
    silent_mode = LaunchConfiguration('silent_mode')
    rtsp_silent_mode = LaunchConfiguration('rtsp_silent_mode')
    
    # Declare the launch options
    declare_display_debug = DeclareLaunchArgument(
        name='display_debug_image',
        default_value='false',
        description='Enable visual display of camera feeds with debug info'
    )
    
    declare_log_to_file = DeclareLaunchArgument(
        name='log_to_file',
        default_value='true',
        description='Log performance metrics to CSV file'
    )
    
    declare_monitor_period = DeclareLaunchArgument(
        name='monitor_period',
        default_value='5.0',
        description='How often (in seconds) to update metrics'
    )
    
    declare_run_duration = DeclareLaunchArgument(
        name='run_duration',
        default_value='30.0',
        description='Duration (in seconds) to run the performance test'
    )
    
    declare_silent_mode = DeclareLaunchArgument(
        name='silent_mode',
        default_value='false',
        description='Run performance monitor without printing continuous messages to the terminal'
    )
    
    declare_rtsp_silent_mode = DeclareLaunchArgument(
        name='rtsp_silent_mode',
        default_value='true',
        description='Run RTSP cameras in silent mode (no terminal output)'
    )
    
    # Include the main multicamera pipeline with silent mode
    camera_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(multicamera_prefix, "launch", "teleop_multicam.launch.py")
        ),
        launch_arguments={
            'silent_mode': rtsp_silent_mode
        }.items()
    )
    
    # Launch the performance monitor
    performance_monitor = Node(
        package='multicamera_pipeline',
        executable='camera_performance_monitor',
        name='camera_performance_monitor',
        output='screen',  # Keep this on screen so we see the countdown and stats
        parameters=[{
            'display_debug_image': display_debug_image,
            'log_to_file': log_to_file,
            'monitor_period_sec': monitor_period,
            'run_duration_sec': run_duration,
            'silent_mode': silent_mode
        }]
    )
    
    # Combine everything into a launch description
    return LaunchDescription([
        declare_display_debug,
        declare_log_to_file,
        declare_monitor_period,
        declare_run_duration,
        declare_silent_mode,
        declare_rtsp_silent_mode,
        camera_pipeline,
        performance_monitor
    ]) 