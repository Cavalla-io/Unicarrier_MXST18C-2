#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys
import subprocess
import argparse
import os

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
        can0_running = can0_exists and 'RUNNING' in result_can0.stdout
        can1_running = can1_exists and 'RUNNING' in result_can1.stdout
        
        # Check if they're configured as CAN interfaces
        can0_is_can = can0_exists and 'mttcan' in result_can0.stdout
        can1_is_can = can1_exists and 'mttcan' in result_can1.stdout
        
        # Check bitrate settings if interfaces exist
        can0_bitrate = '250000' in result_can0.stdout if can0_exists else False
        can1_bitrate = '250000' in result_can1.stdout if can1_exists else False
        
        return {
            'can0_exists': can0_exists,
            'can1_exists': can1_exists,
            'can0_up': can0_up,
            'can1_up': can1_up,
            'can0_running': can0_running,
            'can1_running': can1_running,
            'can0_is_can': can0_is_can,
            'can1_is_can': can1_is_can,
            'can0_bitrate_correct': can0_bitrate,
            'can1_bitrate_correct': can1_bitrate
        }
    except Exception as e:
        print(f"Error checking CAN interfaces: {e}")
        return None

def reset_can_interfaces():
    """Reset CAN interfaces to ensure they're in a good state"""
    print("\n==== Resetting CAN Interfaces ====")
    try:
        # First bring them down
        os.system("sudo ip link set can0 down")
        os.system("sudo ip link set can1 down")
        
        # Then bring them back up with correct settings
        os.system("sudo ip link set can0 up type can bitrate 250000")
        os.system("sudo ip link set can1 up type can bitrate 250000")
        
        # Set queue lengths
        os.system("sudo ip link set can0 txqueuelen 10000")
        os.system("sudo ip link set can1 txqueuelen 10000")
        
        # Give them a moment to stabilize
        time.sleep(1)
        
        # Check if they're running
        results = check_can_interfaces()
        if results and results['can0_running'] and results['can1_running']:
            print("✅ Successfully reset CAN interfaces")
            return True
        else:
            print("❌ Failed to properly reset CAN interfaces")
            return False
    except Exception as e:
        print(f"❌ Error resetting CAN interfaces: {e}")
        return False

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

def perform_physical_can_test():
    """Test physical communication between CAN0 and CAN1 interfaces"""
    print("\n==== Physical CAN Communication Test ====")
    print("Testing direct communication between CAN0 and CAN1")
    
    # Check if both interfaces exist first
    results = check_can_interfaces()
    if not results:
        print("❌ Could not check interfaces")
        return False
    
    if not (results['can0_exists'] and results['can1_exists']):
        print("❌ Both CAN interfaces must exist for this test")
        return False
    
    if not (results['can0_up'] and results['can1_up']):
        print("❌ Both CAN interfaces must be UP for this test")
        return False
        
    if not (results['can0_running'] and results['can1_running']):
        print("❌ Both CAN interfaces must be RUNNING for this test")
        print("   Try running 'sudo python3 src/can_bus/scripts/setup_can.py' to reset them")
        return False
    
    try:
        # Test CAN0 -> CAN1 communication
        print("\nTesting CAN0 -> CAN1 communication:")
        
        # Start candump on can1
        print("Starting candump on can1...")
        candump_process = subprocess.Popen(
            ['candump', 'can1', '-T', '1000'], 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Give candump time to start
        time.sleep(0.5)
        
        # Send message from can0
        test_msg_id = "321"
        test_msg_data = "ABCDEF42"
        print(f"Sending message from can0: ID={test_msg_id}, Data={test_msg_data}")
        
        cansend_result = subprocess.run(
            ['cansend', 'can0', f'{test_msg_id}#{test_msg_data}'],
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True
        )
        
        if cansend_result.returncode != 0:
            print(f"❌ Failed to send message from can0: {cansend_result.stderr}")
            candump_process.terminate()
            return False
        
        # Wait for message to be received
        time.sleep(1)
        
        # Stop candump and get output
        candump_process.terminate()
        candump_out, candump_err = candump_process.communicate(timeout=2)
        
        # Check if message was received
        print(f"candump output: {candump_out}")
        can1_received = test_msg_id in candump_out and any(byte in candump_out for byte in test_msg_data.replace('', ' '))
        
        # Test CAN1 -> CAN0 communication
        print("\nTesting CAN1 -> CAN0 communication:")
        
        # Start candump on can0
        print("Starting candump on can0...")
        candump_process = subprocess.Popen(
            ['candump', 'can0', '-T', '1000'], 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Give candump time to start
        time.sleep(0.5)
        
        # Send message from can1
        test_msg_id = "123"
        test_msg_data = "CAFEBABE"
        print(f"Sending message from can1: ID={test_msg_id}, Data={test_msg_data}")
        
        cansend_result = subprocess.run(
            ['cansend', 'can1', f'{test_msg_id}#{test_msg_data}'],
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True
        )
        
        if cansend_result.returncode != 0:
            print(f"❌ Failed to send message from can1: {cansend_result.stderr}")
            candump_process.terminate()
            return False
        
        # Wait for message to be received
        time.sleep(1)
        
        # Stop candump and get output
        candump_process.terminate()
        can0_out, can0_err = candump_process.communicate(timeout=2)
        
        # Check if message was received
        print(f"candump output: {can0_out}")
        can0_received = test_msg_id in can0_out and any(byte in can0_out for byte in test_msg_data.replace('', ' '))
        
        # Print results
        if can1_received:
            print("✅ CAN1 successfully received message from CAN0")
        else:
            print("❌ CAN1 did not receive message from CAN0")
            
        if can0_received:
            print("✅ CAN0 successfully received message from CAN1")
        else:
            print("❌ CAN0 did not receive message from CAN1")
        
        if can1_received and can0_received:
            print("\n✅ Physical CAN communication test PASSED - bidirectional communication working!")
            return True
        elif can1_received or can0_received:
            print("\n⚠️ Physical CAN communication test PARTIAL - communication only works in one direction")
            return False
        else:
            print("\n❌ Physical CAN communication test FAILED - no communication in either direction")
            print("   Check that the CAN transceiver is properly connected and powered")
            print("   Ensure the two CAN channels are physically connected (CAN0_H to CAN1_H, CAN0_L to CAN1_L)")
            return False
            
    except Exception as e:
        print(f"❌ Error in physical CAN test: {e}")
        return False

def perform_loopback_test(interface='can0'):
    """Perform a loopback test on the specified CAN interface"""
    print(f"\nPerforming loopback test on {interface}...")
    print("NOTE: This test requires physically connecting RX to TX on the CAN transceiver!")
    
    # Save original interface state to restore later
    original_state = None
    try:
        result = subprocess.run(
            ['ip', '-d', 'link', 'show', interface],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        original_state = result.stdout
    except:
        pass
    
    try:
        # Set up interface in loopback mode
        if os.system(f"sudo ip link set {interface} down") != 0:
            print(f"❌ Failed to bring down {interface}")
            return False
            
        if os.system(f"sudo ip link set {interface} up type can bitrate 250000 loopback on") != 0:
            print(f"❌ Failed to set {interface} in loopback mode")
            return False
            
        # Start candump in background, save pid so we can kill it
        print(f"Starting candump on {interface}...")
        candump_process = subprocess.Popen(
            ['candump', interface], 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True
        )
        
        time.sleep(1)  # Give candump time to start
        
        # Send a test message
        test_msg = "DEADBEEF"
        print(f"Sending test message: {test_msg}")
        cansend_result = subprocess.run(
            ['cansend', interface, f'123#{test_msg}'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        if cansend_result.returncode != 0:
            print(f"❌ Failed to send test message: {cansend_result.stderr}")
            # Kill candump process
            candump_process.terminate()
            return False
            
        # Wait a moment for the message to be received
        time.sleep(0.5)
        
        # Kill candump process
        candump_process.terminate()
        candump_out, candump_err = candump_process.communicate(timeout=2)
        
        # Check if our message was received - look for bytes in any format
        print(f"candump output: {candump_out}")
        if "123" in candump_out and ("DEAD" in candump_out or "DE AD BE EF" in candump_out or "DE" in candump_out):
            print(f"✅ Loopback test successful! Received the test message.")
            success = True
        else:
            print(f"❌ Test message not received in loopback mode.")
            success = False
            
        return success
    except Exception as e:
        print(f"❌ Error in loopback test: {e}")
        return False
    finally:
        # Critical: Reset interface to normal mode
        print(f"Resetting {interface} to normal mode...")
        os.system(f"sudo ip link set {interface} down")
        os.system(f"sudo ip link set {interface} up type can bitrate 250000 loopback off")
        time.sleep(0.5)
        
        # If we have the original state and it was RUNNING, make sure it gets back to RUNNING
        if original_state and "RUNNING" in original_state:
            # Force it to RUNNING again if needed
            check_result = subprocess.run(
                ['ip', '-d', 'link', 'show', interface],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            if "RUNNING" not in check_result.stdout:
                print(f"Interface {interface} not RUNNING after test, attempting to fix...")
                os.system(f"sudo ip link set {interface} down")
                os.system(f"sudo ip link set {interface} up type can bitrate 250000")

def check_kernel_modules():
    """Check if required CAN kernel modules are loaded"""
    print("\n==== Checking CAN Kernel Modules ====")
    
    try:
        # Check for required modules
        modules = ['can', 'can_raw', 'mttcan']
        missing_modules = []
        
        for module in modules:
            result = subprocess.run(
                ['lsmod'], 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                text=True
            )
            
            if module not in result.stdout:
                missing_modules.append(module)
        
        if missing_modules:
            print(f"❌ The following required kernel modules are not loaded: {', '.join(missing_modules)}")
            return False
        else:
            print("✅ All required CAN kernel modules are loaded")
            return True
            
    except Exception as e:
        print(f"❌ Error checking kernel modules: {e}")
        return False

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Test the CAN Bus ROS2 node')
    parser.add_argument('--test-all', action='store_true', help='Run all tests')
    parser.add_argument('--check-interfaces', action='store_true', help='Check CAN interfaces')
    parser.add_argument('--check-node', action='store_true', help='Check if CAN bus node is running')
    parser.add_argument('--test-commands', action='store_true', help='Test sending commands')
    parser.add_argument('--test-loopback', action='store_true', help='Perform loopback test (requires TX-RX physical connection)')
    parser.add_argument('--check-modules', action='store_true', help='Check required CAN kernel modules')
    parser.add_argument('--test-physical', action='store_true', help='Test physical communication between CAN0 and CAN1')
    parser.add_argument('--reset', action='store_true', help='Reset CAN interfaces to a good state')
    
    parsed_args = parser.parse_args()
    
    # If no arguments specified, default to running all tests
    if not (parsed_args.test_all or parsed_args.check_interfaces or 
            parsed_args.check_node or parsed_args.test_commands or
            parsed_args.test_loopback or parsed_args.check_modules or
            parsed_args.test_physical or parsed_args.reset):
        parsed_args.test_all = True
    
    # Reset CAN interfaces if requested
    if parsed_args.test_all or parsed_args.reset:
        reset_can_interfaces()
    
    # Check kernel modules
    if parsed_args.test_all or parsed_args.check_modules:
        check_kernel_modules()
    
    # Check CAN interfaces
    if parsed_args.test_all or parsed_args.check_interfaces:
        print("\n==== Checking CAN Interfaces ====")
        results = check_can_interfaces()
        
        if results:
            # Print can0 status
            if results['can0_exists']:
                can0_status = 'UP' if results['can0_up'] else 'DOWN'
                can0_running = 'RUNNING' if results['can0_running'] else 'NOT RUNNING'
                can0_type = 'CAN' if results['can0_is_can'] else 'NOT CAN'
                can0_bitrate = 'CORRECT (250000)' if results['can0_bitrate_correct'] else 'INCORRECT'
                print(f"can0: {can0_status}, {can0_running}, Type: {can0_type}, Bitrate: {can0_bitrate}")
            else:
                print("can0: NOT FOUND")
            
            # Print can1 status
            if results['can1_exists']:
                can1_status = 'UP' if results['can1_up'] else 'DOWN'
                can1_running = 'RUNNING' if results['can1_running'] else 'NOT RUNNING'
                can1_type = 'CAN' if results['can1_is_can'] else 'NOT CAN'
                can1_bitrate = 'CORRECT (250000)' if results['can1_bitrate_correct'] else 'INCORRECT'
                print(f"can1: {can1_status}, {can1_running}, Type: {can1_type}, Bitrate: {can1_bitrate}")
            else:
                print("can1: NOT FOUND")
                
            # Overall status
            if (results['can0_exists'] and results['can0_up'] and results['can0_running'] and results['can0_is_can'] and results['can0_bitrate_correct'] and
                results['can1_exists'] and results['can1_up'] and results['can1_running'] and results['can1_is_can'] and results['can1_bitrate_correct']):
                print("✅ CAN interfaces are properly configured")
            else:
                print("❌ CAN interfaces are not properly configured")
                print("   Run 'sudo python3 src/can_bus/scripts/setup_can.py' to fix")
        else:
            print("❌ Failed to check CAN interfaces")
    
    # Perform physical CAN test
    if parsed_args.test_all or parsed_args.test_physical:
        perform_physical_can_test()
    
    # Perform loopback test
    if parsed_args.test_all or parsed_args.test_loopback:
        print("\n==== Performing CAN Loopback Test ====")
        perform_loopback_test('can0')
        
        # Reset interfaces after loopback test to ensure they're in a good state
        reset_can_interfaces()
    
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
    
    # Final check of interface status after all tests
    if parsed_args.test_all:
        results = check_can_interfaces()
        if results and (results['can0_running'] and results['can1_running']):
            print("\n✅ CAN interfaces still in RUNNING state after tests")
        else:
            print("\n❌ CAN interfaces not in RUNNING state after tests")
            print("   Running reset to fix...")
            reset_can_interfaces()
    
    print("\nTest run completed.")
    print("\nFor additional testing, you can run:")
    print("1. Just the interface check: ./test_can_bus.sh --check-interfaces")
    print("2. Just the physical test: ./test_can_bus.sh --test-physical")
    print("3. Just the loopback test: ./test_can_bus.sh --test-loopback")
    print("4. Just the ROS node commands: ./test_can_bus.sh --test-commands")
    print("5. Reset interfaces if needed: ./test_can_bus.sh --reset")
    print("\nIf physical test fails, make sure the physical connections between CAN0 and CAN1 are correct")
    print("Connect: CAN0_H to CAN1_H, CAN0_L to CAN1_L, and make sure termination resistors are enabled")

if __name__ == "__main__":
    main() 