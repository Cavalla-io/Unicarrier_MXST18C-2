import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
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
    
    # Launch both cameras
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
    
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
