from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_bus',
            executable='can_bus_node',
            name='can_bus_node',
            output='screen',
            emulate_tty=True,
        )
    ]) 