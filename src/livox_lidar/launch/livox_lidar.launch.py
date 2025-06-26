import os
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create launch arguments
    livox_config = DeclareLaunchArgument(
        'livox_config',
        default_value='default',
        description='Livox LiDAR configuration name'
    )
    
    # Define the livox driver path - use the exact path since package isn't in ROS workspace
    livox_driver_path = os.path.join(os.path.expanduser('~'), 'ws_livox')
    
    # Create nodes using ExecuteProcess for the Livox drivers
    livox_driver_nodes = []
    for lidar_name in ['front_left', 'front_right', 'rear']:
        livox_driver_nodes.append(
            ExecuteProcess(
                cmd=[
                    os.path.join(livox_driver_path, 'build', 'livox_ros_driver2'),
                    '--lidar_ip', f'192.168.1.10{1 if lidar_name == "front_left" else 2 if lidar_name == "front_right" else 3}',
                    '--ns', f'/livox/{lidar_name}/data'
                ],
                name=f'livox_driver_{lidar_name}',
                output='screen'
            )
        )
    
    # Add our custom nodes
    nodes = [
        Node(
            package='livox_lidar',
            executable='livox_lidar_node',
            name='livox_lidar_node',
            output='screen'
        ),
        Node(
            package='livox_lidar',
            executable='setup_livox',
            name='livox_setup_node',
            output='screen'
        )
    ]
    
    return launch.LaunchDescription([
        livox_config,
        *livox_driver_nodes,
        *nodes
    ])
