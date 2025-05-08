import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Get package directories
    multicamera_prefix = get_package_share_directory("multicamera_pipeline")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    
    # Get launch parameters
    silent_mode = LaunchConfiguration('silent_mode')
    
    # Use our PoE-optimized configuration file
    params_file = os.path.join(multicamera_prefix, "config", "poe_cameras.yaml")
    
    # Define our two OAK-D Pro cameras
    cams = ["front_camera", "fork_camera"]
    nodes = []
    i = 0.0
    
    # Launch both Luxonis cameras
    for cam_name in cams:
        node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": cam_name,
                "parent_frame": "map",
                "params_file": params_file,
                "cam_pos_y": str(i),
                "rectify_rgb": "false",  # Disable image rectification
                "enable_depth": "false",  # Disable depth processing
                "rsp_use_composition": "false",  # Disable composition
                "use_rviz": "false",  # Disable RViz visualization
                "camera_model": "OAK-D-PRO-POE-NO-IMU",  # Specify custom camera model without IMU/stereo
                "imu_from_descr": "false",  # Disable IMU frames from URDF
                "publish_tf_from_calibration": "false",  # Disable publishing TF from calibration
            }.items(),
        )
        nodes.append(node)
        i = i + 0.3  # Larger spacing between cameras
    
    # Launch RTSP cameras as direct nodes
    rtsp_nodes = [
        Node(
            package='multicamera_pipeline',
            executable='rtsp_stream_node',
            name='rtsp_camera_1',
            parameters=[{
                'rtsp_url': 'rtsp://192.168.2.250:554/stream',
                'topic_name': 'rtsp_camera1/image_raw',
                'frame_rate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'pipeline_type': 4,  # Use TCP pipeline for better reliability
                'silent_mode': silent_mode,
            }],
            output='log',  # Redirect output to log files instead of terminal
        ),
        Node(
            package='multicamera_pipeline',
            executable='rtsp_stream_node',
            name='rtsp_camera_2',
            parameters=[{
                'rtsp_url': 'rtsp://192.168.2.244:554/stream',
                'topic_name': 'rtsp_camera2/image_raw',
                'frame_rate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'pipeline_type': 4,  # Use TCP pipeline for better reliability
                'silent_mode': silent_mode,
            }],
            output='log',  # Redirect output to log files instead of terminal
        ),
    ]
    
    # Add the RTSP nodes to the launch list
    nodes.extend(rtsp_nodes)
    
    return nodes


def generate_launch_description():
    # Declare launch arguments
    silent_arg = DeclareLaunchArgument(
        'silent_mode',
        default_value='true',
        description='Run in silent mode with minimal terminal output'
    )
    
    return LaunchDescription(
        [
            silent_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
