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

from can_bus.can_bus_setup import can_startup, can_shutdown
from can_bus.can_bus_lift_msg import LiftController

class CanBusNode(Node):
    def __init__(self):
        super().__init__('can_bus_node')
        
        # Configure CAN interfaces - using can2/can3 to avoid conflicts with existing interfaces
        self.CAN2_INTERFACE = 'can2'
        self.CAN3_INTERFACE = 'can3'
        
        # Set up CAN bus interfaces
        self.bus_can2 = can.interface.Bus(channel=self.CAN2_INTERFACE, bustype='socketcan')
        self.bus_can3 = can.interface.Bus(channel=self.CAN3_INTERFACE, bustype='socketcan')
        
        # Initialize the lift controller - using can3 for lift control
        self.lift_controller = LiftController(self.bus_can3, 0x1A0)
        
        # Global flags and state
        self.passthrough_active = True
        self.is_running = True
        self.blocked_ids = set()  # Set of blocked IDs
        self.blocked_ids_lock = threading.Lock()  # Lock to protect access to blocked_ids
        
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
        
        # Start the passthrough loop in a separate thread
        self.passthrough_thread = threading.Thread(target=self.passthrough_loop)
        self.passthrough_thread.daemon = True
        self.passthrough_thread.start()
        
        # Set up signal handlers for graceful shutdown
        self.setup_signal_handlers()
        
        self.get_logger().info("CAN Bus Node started and ready to receive commands")
        self.get_logger().info(f"Using CAN interfaces: {self.CAN2_INTERFACE} and {self.CAN3_INTERFACE}")
    
    def setup_signal_handlers(self):
        """Set up signal handlers for graceful shutdown"""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle termination signals gracefully"""
        self.get_logger().info(f"Received signal {sig}, shutting down gracefully...")
        self.is_running = False
        time.sleep(0.5)  # Give time for threads to clean up
        rclpy.shutdown()
        sys.exit(0)
    
    def command_callback(self, msg):
        """Handle incoming commands on the topic"""
        command = msg.data.strip()
        self.get_logger().info(f"Received command: {command}")
        
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
            self.lift_controller.stop_lift()
            self.passthrough_active = True
            
        elif command == "STATUS_CHECK":
            self.passthrough_active = False
            self.send_can_message(self.bus_can2, 0x200, [0x00])  # Example status check message
            self.passthrough_active = True
            
        elif command == "PASSTHROUGH_ON":
            self.passthrough_active = True
            self.get_logger().info("CAN bus passthrough enabled")
            
        elif command == "PASSTHROUGH_OFF":
            self.passthrough_active = False
            self.get_logger().info("CAN bus passthrough disabled")
            
        else:
            self.get_logger().warn(f"Unknown command: {command}")
    
    def stop_other_operations(self):
        """Stop any ongoing operations before starting a new one."""
        self.lift_controller.stop_lift()
    
    def send_can_message(self, bus, arbitration_id, data):
        """Send a CAN message on the specified bus."""
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
        while self.is_running:
            if self.passthrough_active:
                try:
                    # Non-blocking receive from CAN2
                    can2_msg = self.bus_can2.recv(timeout=0)
                    if can2_msg:
                        with self.blocked_ids_lock:
                            if can2_msg.arbitration_id not in self.blocked_ids:
                                self.bus_can3.send(can2_msg)  # Forward to CAN3

                    # Non-blocking receive from CAN3
                    can3_msg = self.bus_can3.recv(timeout=0)
                    if can3_msg:
                        with self.blocked_ids_lock:
                            if can3_msg.arbitration_id not in self.blocked_ids:
                                self.bus_can2.send(can3_msg)  # Forward to CAN2
                except can.CanError as e:
                    self.get_logger().error(f"CAN passthrough error: {e}")
            else:
                # Sleep briefly to reduce CPU usage when passthrough is inactive
                time.sleep(0.01)

def main(args=None):
    # Ensure CAN bus setup is performed
    can_startup()
    
    rclpy.init(args=args)
    can_bus_node = CanBusNode()
    
    try:
        rclpy.spin(can_bus_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the lift controller
        can_bus_node.lift_controller.stop_lift()
        # Clean shutdown
        can_bus_node.is_running = False
        can_shutdown()
        # Destroy the node
        can_bus_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 