#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys
import subprocess
import argparse

class CANBusTestNode(Node):
    def __init__(self):
        super().__init__('can_bus_test_node')
        
        # Create publisher for sending commands
        self.command_publisher = self.create_publisher(
            String,
            '/forklift/can_command',
            10
        )
        
        # Log startup
        self.get_logger().info("CAN Bus Test Node started")
        
    def send_command(self, command_str):
        """Send a command to the CAN bus node"""
        msg = String()
        msg.data = command_str
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Sent command: {command_str}")

def check_can_interfaces():
    """Check if CAN interfaces are properly initialized"""
    try:
        # Run 'ip -d link show can0' to check can0 interface
        result_can0 = subprocess.run(
            ['ip', '-d', 'link', 'show', 'can0'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Run 'ip -d link show can1' to check can1 interface
        result_can1 = subprocess.run(
            ['ip', '-d', 'link', 'show', 'can1'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Check if interfaces exist and are UP
        can0_exists = result_can0.returncode == 0
        can1_exists = result_can1.returncode == 0
        can0_up = can0_exists and 'state UP' in result_can0.stdout
        can1_up = can1_exists and 'state UP' in result_can1.stdout
        
        # Check bitrate settings if interfaces exist
        can0_bitrate = '250000' in result_can0.stdout if can0_exists else False
        can1_bitrate = '250000' in result_can1.stdout if can1_exists else False
        
        return {
            'can0_exists': can0_exists,
            'can1_exists': can1_exists,
            'can0_up': can0_up,
            'can1_up': can1_up,
            'can0_bitrate_correct': can0_bitrate,
            'can1_bitrate_correct': can1_bitrate
        }
    except Exception as e:
        print(f"Error checking CAN interfaces: {e}")
        return None

def check_ros_node_running():
    """Check if the CAN bus ROS2 node is running"""
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        return '/can_bus_node' in result.stdout
    except Exception as e:
        print(f"Error checking if ROS2 node is running: {e}")
        return False

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Test the CAN Bus ROS2 node')
    parser.add_argument('--test-all', action='store_true', help='Run all tests')
    parser.add_argument('--check-interfaces', action='store_true', help='Check CAN interfaces')
    parser.add_argument('--check-node', action='store_true', help='Check if CAN bus node is running')
    parser.add_argument('--test-commands', action='store_true', help='Test sending commands')
    
    parsed_args = parser.parse_args()
    
    # If no arguments specified, default to running all tests
    if not (parsed_args.test_all or parsed_args.check_interfaces or 
            parsed_args.check_node or parsed_args.test_commands):
        parsed_args.test_all = True
    
    # Check CAN interfaces
    if parsed_args.test_all or parsed_args.check_interfaces:
        print("\n==== Checking CAN Interfaces ====")
        results = check_can_interfaces()
        
        if results:
            # Print can0 status
            if results['can0_exists']:
                can0_status = 'UP' if results['can0_up'] else 'DOWN'
                can0_bitrate = 'CORRECT (250000)' if results['can0_bitrate_correct'] else 'INCORRECT'
                print(f"can0: {can0_status}, Bitrate: {can0_bitrate}")
            else:
                print("can0: NOT FOUND")
            
            # Print can1 status
            if results['can1_exists']:
                can1_status = 'UP' if results['can1_up'] else 'DOWN'
                can1_bitrate = 'CORRECT (250000)' if results['can1_bitrate_correct'] else 'INCORRECT'
                print(f"can1: {can1_status}, Bitrate: {can1_bitrate}")
            else:
                print("can1: NOT FOUND")
                
            # Overall status
            if (results['can0_exists'] and results['can0_up'] and results['can0_bitrate_correct'] and
                results['can1_exists'] and results['can1_up'] and results['can1_bitrate_correct']):
                print("✅ CAN interfaces are properly configured")
            else:
                print("❌ CAN interfaces are not properly configured")
        else:
            print("❌ Failed to check CAN interfaces")
    
    # Check if ROS2 node is running
    if parsed_args.test_all or parsed_args.check_node:
        print("\n==== Checking CAN Bus ROS2 Node ====")
        node_running = check_ros_node_running()
        
        if node_running:
            print("✅ CAN Bus ROS2 node is running")
        else:
            print("❌ CAN Bus ROS2 node is not running")
            print("   Try running: ros2 launch can_bus can_bus.launch.py")
            
            # If the node isn't running and we need to test commands, exit early
            if parsed_args.test_all or parsed_args.test_commands:
                print("\n❌ Cannot test commands because the ROS2 node is not running")
                return
    
    # Test sending commands to the node
    if parsed_args.test_all or parsed_args.test_commands:
        if not (parsed_args.test_all or parsed_args.check_node) or check_ros_node_running():
            print("\n==== Testing Command Sending ====")
            
            # Initialize the ROS2 system
            rclpy.init(args=args)
            test_node = CANBusTestNode()
            
            # Test command sequence
            print("Sending test commands sequence...")
            
            # First enable passthrough explicitly
            test_node.send_command("PASSTHROUGH_ON")
            time.sleep(1.0)
            
            # Test status check
            print("Testing STATUS_CHECK command...")
            test_node.send_command("STATUS_CHECK")
            time.sleep(1.0)
            
            # Test lift up (briefly)
            print("Testing LIFT_UP command (2 seconds)...")
            test_node.send_command("LIFT_UP")
            time.sleep(2.0)
            
            # Stop lift
            print("Testing STOP command...")
            test_node.send_command("STOP")
            time.sleep(1.0)
            
            # Test lift down (briefly)
            print("Testing LIFT_DOWN command (2 seconds)...")
            test_node.send_command("LIFT_DOWN")
            time.sleep(2.0)
            
            # Stop lift again
            test_node.send_command("STOP")
            time.sleep(1.0)
            
            # Test passthrough off/on
            print("Testing PASSTHROUGH_OFF command...")
            test_node.send_command("PASSTHROUGH_OFF")
            time.sleep(1.0)
            
            print("Testing PASSTHROUGH_ON command...")
            test_node.send_command("PASSTHROUGH_ON")
            time.sleep(1.0)
            
            # Clean up
            test_node.destroy_node()
            rclpy.shutdown()
            
            print("✅ Command testing complete")
    
    print("\nTest run completed.")

if __name__ == "__main__":
    main() 