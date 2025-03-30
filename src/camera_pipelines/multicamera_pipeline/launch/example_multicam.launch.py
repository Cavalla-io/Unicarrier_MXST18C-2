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
    cams = ["oak_d_pro_1", "oak_d_pro_2"]
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
