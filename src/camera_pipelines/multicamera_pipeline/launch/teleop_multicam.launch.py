import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Get package directories
    multicamera_prefix = get_package_share_directory("multicamera_pipeline")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    
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
    
    # Add RTSP Camera streams using launch includes
    # Define RTSP camera configurations
    rtsp_cameras = [
        {
            "rtsp_url": "rtsp://192.168.2.250:554/stream",
            "topic_name": "rtsp_camera1/image_raw",
            "frame_rate": "30.0",
            "pipeline_type": "2",  # Use alternative pipeline
        },
        {
            "rtsp_url": "rtsp://192.168.2.244:554/stream",
            "topic_name": "rtsp_camera2/image_raw",
            "frame_rate": "30.0",
            "pipeline_type": "4",  # Use TCP pipeline
        },
    ]
    
    # Launch RTSP cameras using launch include
    for idx, rtsp_config in enumerate(rtsp_cameras):
        rtsp_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(multicamera_prefix, "launch", "rtsp_stream.launch.py")
            ),
            launch_arguments={
                "rtsp_url": rtsp_config["rtsp_url"],
                "topic_name": rtsp_config["topic_name"],
                "frame_rate": rtsp_config["frame_rate"],
                "image_width": "640",
                "image_height": "480",
                "pipeline_type": rtsp_config["pipeline_type"],
            }.items(),
        )
        nodes.append(rtsp_node)
    
    # Add direct RTSP nodes as an alternative option
    # These can work even if there are issues with the launch file includes
    rtsp_direct_nodes = [
        Node(
            package='multicamera_pipeline',
            executable='rtsp_stream_node',
            name='rtsp_direct_1',
            parameters=[{
                'rtsp_url': 'rtsp://192.168.2.250:554/stream',
                'topic_name': 'rtsp_direct1/image_raw',
                'frame_rate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'pipeline_type': 1,
            }],
            output='screen',
        ),
        Node(
            package='multicamera_pipeline',
            executable='rtsp_stream_node',
            name='rtsp_direct_2',
            parameters=[{
                'rtsp_url': 'rtsp://192.168.2.244:554/stream',
                'topic_name': 'rtsp_direct2/image_raw',
                'frame_rate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'pipeline_type': 3,  # Try hardware acceleration if available
            }],
            output='screen',
        ),
    ]
    
    # Add the direct nodes to the launch list
    nodes.extend(rtsp_direct_nodes)
    
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
