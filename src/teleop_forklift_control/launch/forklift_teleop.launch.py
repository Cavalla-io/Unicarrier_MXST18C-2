#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for the forklift teleoperation system."""
    
    # Launch arguments
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Run the wheel tracker in simulation mode'
    )
    
    # Main controller node
    main_controller_node = Node(
        package='teleop_forklift_control',
        executable='main',
        name='forklift_controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[]
    )
    
    # Wheel tracker node with simulation mode
    wheel_tracker_sim_node = Node(
        package='teleop_forklift_control',
        executable='wheel_tracker',
        name='wheel_tracker_node',
        output='screen',
        emulate_tty=True,
        arguments=['--sim'],
        condition=IfCondition(LaunchConfiguration('sim_mode'))
    )
    
    # Wheel tracker node without simulation mode
    wheel_tracker_real_node = Node(
        package='teleop_forklift_control',
        executable='wheel_tracker',
        name='wheel_tracker_node',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('sim_mode'))
    )
    
    return LaunchDescription([
        sim_mode_arg,
        main_controller_node,
        wheel_tracker_sim_node,
        wheel_tracker_real_node
    ]) 