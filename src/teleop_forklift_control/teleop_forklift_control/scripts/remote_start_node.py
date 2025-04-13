#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time
import signal
import sys

class RemoteStartNode(Node):
    def __init__(self):
        super().__init__('remote_start_node')
        
        # Initialize variables
        self.running = True
        self.lock = threading.Lock()
        self.current_state = "stop"  # Default state is stop
        
        # Initialize serial connection to steering board
        try:
            self.serial_port = serial.Serial(
                port='/dev/steering',
                baudrate=115200,  # Adjust baudrate as needed for your steering board
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.get_logger().info("Successfully connected to steering board on /dev/steering")
        except Exception as e:
            self.get_logger().error(f"Error connecting to steering board: {e}")
            self.serial_port = None
        
        # Create a subscriber for the remote start/stop commands
        self.start_stop_subscriber = self.create_subscription(
            String,  # Using String message type for start/stop commands
            '/forklift/remote_start',
            self.remote_start_callback,
            10  # QoS history depth
        )
        
        # Set up signal handler for graceful shutdown
        self.setup_signal_handlers()
        
        # Send initial stop command to ensure safe state
        self.send_command_to_steering_board("stop")
        
        self.get_logger().info("Remote start node started")
    
    def setup_signal_handlers(self):
        """Set up signal handlers for graceful shutdown"""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle termination signals gracefully"""
        self.get_logger().info(f"Received signal {sig}, shutting down gracefully...")
        self.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    def remote_start_callback(self, msg):
        """Handle incoming remote start/stop commands"""
        command = msg.data.lower().strip()
        
        if command in ["start", "stop"]:
            with self.lock:
                if command != self.current_state:
                    self.current_state = command
                    self.get_logger().info(f"Received {command} command")
                    self.send_command_to_steering_board(command)
        else:
            self.get_logger().warn(f"Received unknown command: {command}")
    
    def send_command_to_steering_board(self, command):
        """Send command to the steering board via serial"""
        if not self.serial_port:
            self.get_logger().error(f"Cannot send {command} command: Serial port not connected")
            return
        
        try:
            if command == "start":
                # Format the start command for your steering board
                self.serial_port.write(b"START\n")
                self.get_logger().info("Sent START command to steering board")
            elif command == "stop":
                # Format the stop command for your steering board
                self.serial_port.write(b"STOP\n")
                self.get_logger().info("Sent STOP command to steering board")
        except Exception as e:
            self.get_logger().error(f"Error sending command to steering board: {e}")
    
    def shutdown(self):
        """Clean up resources when shutting down"""
        self.get_logger().info("Shutting down remote start node...")
        self.running = False
        
        # Send a final STOP command for safety
        self.send_command_to_steering_board("stop")
        
        # Close serial connection
        if self.serial_port:
            try:
                self.serial_port.close()
                self.get_logger().info("Closed serial connection to steering board")
            except Exception as e:
                self.get_logger().error(f"Error closing serial connection: {e}")

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create the node
    remote_start = None
    
    try:
        # Create and initialize the node
        remote_start = RemoteStartNode()
        
        # Spin the node
        rclpy.spin(remote_start)
    except KeyboardInterrupt:
        if remote_start:
            remote_start.get_logger().info("Keyboard interrupt received, shutting down.")
    except Exception as e:
        print(f"Error in remote start node: {e}")
    finally:
        # Ensure proper shutdown
        if remote_start:
            remote_start.shutdown()
            remote_start.destroy_node()
        
        # Ensure ROS is shut down
        if rclpy.ok():
            rclpy.shutdown()
            
        print("Remote start node has exited")

if __name__ == '__main__':
    main() 