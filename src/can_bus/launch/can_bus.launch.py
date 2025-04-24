from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get path to the source scripts directory (fallback to installed path)
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    script_path = os.path.join(package_dir, 'scripts', 'setup_can.py')
    
    # If not found in source, try the installed location
    if not os.path.exists(script_path):
        package_share = get_package_share_directory('can_bus')
        script_path = os.path.join(package_share, 'scripts', 'setup_can.py')
    
    # Check if the script exists
    if not os.path.exists(script_path):
        return LaunchDescription([
            LogInfo(msg=f"Warning: Setup script not found at {script_path}"),
            LogInfo(msg="Running CAN Bus node without setup script"),
            Node(
                package='can_bus',
                executable='can_bus_node',
                name='can_bus_node',
                output='screen',
                emulate_tty=True,
            )
        ])
    
    # Create actions with the correct path
    setup_can_cmd = ExecuteProcess(
        cmd=["python3", script_path],
        name='setup_can',
        output='screen',
    )
    
    can_bus_node = Node(
        package='can_bus',
        executable='can_bus_node',
        name='can_bus_node',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # First set up CAN interfaces with sudo
        setup_can_cmd,
        # Then start the CAN bus node
        can_bus_node
    ]) 