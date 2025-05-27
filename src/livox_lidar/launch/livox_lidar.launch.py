import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create launch arguments
    livox_config = DeclareLaunchArgument(
        'livox_config',
        default_value='default',
        description='Livox LiDAR configuration name'
    )
    
    # Create nodes
    livox_driver_nodes = []
    for lidar_name in ['front_left', 'front_right', 'rear']:
        livox_driver_nodes.append(
            Node(
                package='livox_ros_driver2',
                executable='livox_driver',
                name=f'livox_driver_{lidar_name}',
                parameters=[
                    {'lidar_ip': f'192.168.1.10{1 if lidar_name == "front_left" else 2 if lidar_name == "front_right" else 3}'}
                ],
                remappings=[
                    ('/livox/lidar', f'/livox/{lidar_name}/data')
                ],
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
    
    # Add dependencies
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'livox_ros_driver2',
            default_value='livox_ros_driver2',
            description='Livox ROS2 Driver package'
        ),
        *nodes
    ])
