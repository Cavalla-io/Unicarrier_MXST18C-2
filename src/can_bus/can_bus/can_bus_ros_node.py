#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
import threading
import time
import signal
import sys
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from can_bus.can_bus_setup import can_startup, can_shutdown, check_can_state, check_interface_exists
from can_bus.can_bus_lift_msg import LiftController

class CanBusNode(Node):
    def __init__(self):
        super().__init__('can_bus_node')
        
        # Configure CAN interfaces - using can2/can3 to avoid conflicts with existing interfaces
        self.CAN2_INTERFACE = 'can2'
        self.CAN3_INTERFACE = 'can3'
        
        # Set up CAN interfaces using sudo
        self.get_logger().info("Setting up CAN interfaces...")
        can_startup()
        
        # Set up CAN bus interfaces with error handling
        self.get_logger().info(f"Connecting to CAN interfaces {self.CAN2_INTERFACE} and {self.CAN3_INTERFACE}...")
        self.connect_to_can_interfaces()
        
        # Create a QoS profile with BEST_EFFORT reliability
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create a subscriber for CAN bus commands
        self.command_subscriber = self.create_subscription(
            String,
            '/forklift/can_command',
            self.command_callback,
            best_effort_qos
        )
        
        # Global flags and state
        self.passthrough_active = True
        self.is_running = True
        self.blocked_ids = set()  # Set of blocked IDs
        self.blocked_ids_lock = threading.Lock()  # Lock to protect access to blocked_ids
        
        # Start the passthrough loop in a separate thread only if CAN interfaces are available
        if hasattr(self, 'bus_can2') and hasattr(self, 'bus_can3'):
            self.passthrough_thread = threading.Thread(target=self.passthrough_loop)
            self.passthrough_thread.daemon = True
            self.passthrough_thread.start()
            self.get_logger().info("CAN Bus Node started with active passthrough")
        else:
            self.get_logger().warn("Running in limited functionality mode - CAN passthrough disabled")
            
        # Set up signal handlers for graceful shutdown
        self.setup_signal_handlers()
    
    def connect_to_can_interfaces(self):
        """Connect to CAN interfaces with error handling"""
        try:
            # First check if interfaces exist and are up
            can2_exists = check_interface_exists(self.CAN2_INTERFACE)
            can3_exists = check_interface_exists(self.CAN3_INTERFACE)
            
            if not can2_exists or not can3_exists:
                self.get_logger().error(f"Missing CAN interfaces. Can2 exists: {can2_exists}, Can3 exists: {can3_exists}")
                self.get_logger().warn("Only limited functionality will be available.")
                return
            
            # Check if interfaces are up
            can2_ready, _ = check_can_state(self.CAN2_INTERFACE)
            can3_ready, _ = check_can_state(self.CAN3_INTERFACE)
            
            if not can2_ready or not can3_ready:
                self.get_logger().warn("One or more CAN interfaces are not UP.")
                
            # Try to connect to the interfaces
            try:
                self.bus_can2 = can.interface.Bus(channel=self.CAN2_INTERFACE, bustype='socketcan')
                self.get_logger().info(f"Connected to {self.CAN2_INTERFACE}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to {self.CAN2_INTERFACE}: {e}")
                
            try:
                self.bus_can3 = can.interface.Bus(channel=self.CAN3_INTERFACE, bustype='socketcan')
                self.get_logger().info(f"Connected to {self.CAN3_INTERFACE}")
                
                # Initialize the lift controller only if can3 is available
                self.lift_controller = LiftController(self.bus_can3, 0x1A0)
                self.get_logger().info("Lift controller initialized")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to {self.CAN3_INTERFACE}: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Error setting up CAN interfaces: {e}")
    
    def setup_signal_handlers(self):
        """Set up signal handlers for graceful shutdown"""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle termination signals gracefully"""
        self.get_logger().info(f"Received signal {sig}, shutting down gracefully...")
        self.is_running = False
        self.cleanup()
        time.sleep(0.5)  # Give time for threads to clean up
        rclpy.shutdown()
        sys.exit(0)
    
    def cleanup(self):
        """Clean up resources before shutting down"""
        self.get_logger().info("Cleaning up resources...")
        self.is_running = False
        if hasattr(self, 'lift_controller'):
            try:
                self.lift_controller.stop_lift()
            except Exception as e:
                self.get_logger().error(f"Error stopping lift: {e}")
        can_shutdown()
    
    def command_callback(self, msg):
        """Handle incoming commands on the topic"""
        command = msg.data.strip()
        self.get_logger().info(f"Received command: {command}")
        
        # Verify we have the required resources to execute commands
        if not hasattr(self, 'lift_controller') and command in ["LIFT_UP", "LIFT_DOWN", "LIFT_FAST_DOWN", "STOP"]:
            self.get_logger().error(f"Cannot execute {command} - Lift controller not initialized")
            return
        
        try:
            if command == "LIFT_UP":
                self.passthrough_active = False
                self.stop_other_operations()
                with self.blocked_ids_lock:
                    self.blocked_ids.update({0x1A0})  # Block IDs during lifting
                self.lift_controller.start_lift("UP")
                with self.blocked_ids_lock:
                    self.blocked_ids.difference_update({0x1A0})  # Unblock IDs after lifting
                self.passthrough_active = True
                
            elif command == "LIFT_DOWN":
                self.passthrough_active = False
                self.stop_other_operations()
                with self.blocked_ids_lock:
                    self.blocked_ids.update({0x1A0})  # Block IDs during lifting
                self.lift_controller.start_lift("DOWN")
                with self.blocked_ids_lock:
                    self.blocked_ids.difference_update({0x1A0})  # Unblock IDs after lifting
                self.passthrough_active = True
                
            elif command == "LIFT_FAST_DOWN":
                self.passthrough_active = False
                self.stop_other_operations()
                with self.blocked_ids_lock:
                    self.blocked_ids.update({0x1A0})  # Block IDs during lifting
                # Implement fast down if needed or use regular down
                self.lift_controller.start_lift("DOWN")
                with self.blocked_ids_lock:
                    self.blocked_ids.difference_update({0x1A0})  # Unblock IDs after lifting
                self.passthrough_active = True
                
            elif command == "STOP":
                if hasattr(self, 'lift_controller'):
                    self.lift_controller.stop_lift()
                self.passthrough_active = True
                
            elif command == "STATUS_CHECK":
                if hasattr(self, 'bus_can2'):
                    self.passthrough_active = False
                    self.send_can_message(self.bus_can2, 0x200, [0x00])  # Example status check message
                    self.passthrough_active = True
                else:
                    self.get_logger().error("Cannot perform STATUS_CHECK - CAN2 interface not available")
                
            elif command == "PASSTHROUGH_ON":
                self.passthrough_active = True
                self.get_logger().info("CAN bus passthrough enabled")
                
            elif command == "PASSTHROUGH_OFF":
                self.passthrough_active = False
                self.get_logger().info("CAN bus passthrough disabled")
                
            else:
                self.get_logger().warn(f"Unknown command: {command}")
        except Exception as e:
            self.get_logger().error(f"Error executing command {command}: {e}")
            # Re-enable passthrough in case of error
            self.passthrough_active = True
    
    def stop_other_operations(self):
        """Stop any ongoing operations before starting a new one."""
        if hasattr(self, 'lift_controller'):
            self.lift_controller.stop_lift()
    
    def send_can_message(self, bus, arbitration_id, data):
        """Send a CAN message on the specified bus."""
        if not bus:
            self.get_logger().error(f"Cannot send message - bus not available")
            return
            
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            bus.send(msg)
            self.get_logger().debug(f"Message sent on {bus.channel_info}: {msg}")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send message on {bus.channel_info}: {e}")
    
    def passthrough_loop(self):
        """
        Continuously pass messages between CAN2 and CAN3 when passthrough is active,
        with filtering for blocked IDs.
        """
        can2_error_count = 0
        can3_error_count = 0
        max_errors = 5  # Maximum consecutive errors before warning
        
        while self.is_running:
            if self.passthrough_active and hasattr(self, 'bus_can2') and hasattr(self, 'bus_can3'):
                try:
                    # Non-blocking receive from CAN2
                    can2_msg = self.bus_can2.recv(timeout=0.01)  # Small timeout to reduce CPU usage
                    if can2_msg:
                        with self.blocked_ids_lock:
                            if can2_msg.arbitration_id not in self.blocked_ids:
                                self.bus_can3.send(can2_msg)  # Forward to CAN3
                    can2_error_count = 0  # Reset error count on success
                except can.CanError as e:
                    can2_error_count += 1
                    if can2_error_count >= max_errors:
                        self.get_logger().warn(f"Multiple CAN2 errors: {e}")
                        can2_error_count = 0  # Reset after logging
                    time.sleep(0.1)  # Sleep to avoid flooding with errors
                
                try:
                    # Non-blocking receive from CAN3
                    can3_msg = self.bus_can3.recv(timeout=0.01)  # Small timeout to reduce CPU usage
                    if can3_msg:
                        with self.blocked_ids_lock:
                            if can3_msg.arbitration_id not in self.blocked_ids:
                                self.bus_can2.send(can3_msg)  # Forward to CAN2
                    can3_error_count = 0  # Reset error count on success
                except can.CanError as e:
                    can3_error_count += 1
                    if can3_error_count >= max_errors:
                        self.get_logger().warn(f"Multiple CAN3 errors: {e}")
                        can3_error_count = 0  # Reset after logging
                    time.sleep(0.1)  # Sleep to avoid flooding with errors
            else:
                # Sleep a bit longer when passthrough is not active or interfaces not available
                time.sleep(0.1)

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and run the node
        can_bus_node = CanBusNode()
        rclpy.spin(can_bus_node)
    except KeyboardInterrupt:
        print("User interrupted, shutting down...")
    except Exception as e:
        print(f"Error running CAN Bus Node: {e}")
    finally:
        # Ensure clean shutdown
        if 'can_bus_node' in locals():
            can_bus_node.cleanup()
            can_bus_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 