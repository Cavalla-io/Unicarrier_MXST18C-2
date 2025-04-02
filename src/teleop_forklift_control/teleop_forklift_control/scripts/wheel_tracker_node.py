#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
import time
import threading
import signal
import sys
from std_msgs.msg import Float32

# Import the existing tracking logic from wheel_tracker.py
from teleop_forklift_control.controller_depends.wheel_tracker import (
    CAN_CHANNEL, CAN_BITRATE, TARGET_FRAME_ID, translate_encoder_to_angle
)

class WheelTrackerNode(Node):
    def __init__(self, sim_mode=False):
        super().__init__('wheel_tracker_node')
        # Initialize variables
        self.angle = 0.0
        self.raw_value = 0
        self.running = True
        self.lock = threading.Lock()
        self.sim_mode = sim_mode
        self.shutdown_requested = False
        
        # Create a publisher for the wheel angle
        self.wheel_angle_publisher = self.create_publisher(
            Float32,
            '/forklift/wheel_angle',
            10  # QoS history depth
        )

        # Initialize the CAN bus if not in simulation mode
        if not self.sim_mode:
            try:
                self.bus = can.interface.Bus(bustype='socketcan', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
                self.get_logger().info(f"Successfully initialized CAN bus on {CAN_CHANNEL}")
                
                # Create a thread for reading CAN messages
                self.can_thread = threading.Thread(target=self._read_can_loop, daemon=True)
                self.can_thread.start()
            except Exception as e:
                self.get_logger().error(f"Error initializing CAN bus: {e}")
                self.get_logger().info("Running in simulation mode instead")
                self.sim_mode = True
        
        if self.sim_mode:
            self.get_logger().info("Running in simulation mode - wheel angles will be simulated")
            
        # Create a timer for publishing the wheel angle (regardless of CAN or sim mode)
        self.publish_timer = self.create_timer(0.1, self.publish_angle)  # 10 Hz
        self.sim_timer = None
        
        if self.sim_mode:
            # Create a timer to simulate steering changes in simulation mode
            self.sim_angle = 0.0
            self.sim_direction = 1  # 1 for right, -1 for left
            self.sim_timer = self.create_timer(0.1, self.simulate_steering)
            
        # Set up a handler to catch termination signals
        self.setup_signal_handlers()
            
        self.get_logger().info("Wheel tracker node started")
        
    def setup_signal_handlers(self):
        """Set up signal handlers for graceful shutdown"""
        # Register the signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        """Handle termination signals gracefully"""
        self.get_logger().info(f"Received signal {sig}, shutting down gracefully...")
        self.shutdown_requested = True
        self.running = False
        
        # Clean up resources
        self.shutdown()
        
        # Let ROS2 know we're exiting cleanly
        rclpy.shutdown()
        
        # Exit with a successful status code
        sys.exit(0)
    
    def _read_can_loop(self):
        """Background thread function to continuously read CAN messages"""
        self.get_logger().info(f"Listening for CAN frame 0x{TARGET_FRAME_ID:X} on channel {CAN_CHANNEL}...")
        
        while self.running and rclpy.ok():
            try:
                # Use a short timeout to allow checking running flag frequently
                message = self.bus.recv(timeout=0.1)
                if message is None:
                    continue

                if message.arbitration_id == TARGET_FRAME_ID:
                    # Assuming the encoder value is in the first byte of the data payload.
                    with self.lock:
                        self.raw_value = message.data[3]
                        self.angle = translate_encoder_to_angle(self.raw_value)
            except Exception as e:
                if self.running:  # Only log errors if we're still supposed to be running
                    self.get_logger().error(f"Error reading CAN message: {e}")
                time.sleep(0.1)
    
    def simulate_steering(self):
        """Simulate steering angles for testing without CAN hardware"""
        if not self.running:
            return
            
        # Simple simulation: oscillate between -30 and +30 degrees
        with self.lock:
            self.angle += self.sim_direction * 2.0  # Change by 2 degrees each step
            if abs(self.angle) > 30.0:
                self.sim_direction *= -1  # Reverse direction at extremes
                self.angle = 30.0 if self.angle > 0 else -30.0
                
            # Calculate a simulated raw value (just for display purposes)
            self.raw_value = int(((self.angle * (256.0 / 360)) + 0xFF) % 256)
    
    def publish_angle(self):
        """Publish the current wheel angle"""
        if not self.running:
            return
            
        with self.lock:
            angle = self.angle
            raw = self.raw_value
        
        # Create and publish the message
        msg = Float32()
        msg.data = float(angle)
        self.wheel_angle_publisher.publish(msg)
        
        # Log occasionally (every ~5 seconds)
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
            if self.log_counter >= 50:  # 50 * 0.1s = 5s
                mode_str = "[SIM]" if self.sim_mode else "[CAN]"
                # self.get_logger().info(f"{mode_str} Current wheel angle: {angle:.2f} degrees (Raw: 0x{raw:02X})")
                self.log_counter = 0
        else:
            self.log_counter = 0
    
    def shutdown(self):
        """Clean up resources when shutting down"""
        # Prevent multiple shutdowns
        if hasattr(self, '_shutdown_complete') and self._shutdown_complete:
            return
            
        self.get_logger().info("Shutting down wheel tracker node...")
        self.running = False
        
        # Cancel all timers
        if hasattr(self, 'publish_timer'):
            self.destroy_timer(self.publish_timer)
        if hasattr(self, 'sim_timer') and self.sim_timer:
            self.destroy_timer(self.sim_timer)
        
        # Wait for threads to finish
        if hasattr(self, 'can_thread') and self.can_thread and self.can_thread.is_alive():
            self.get_logger().info("Waiting for CAN thread to terminate...")
            self.can_thread.join(timeout=2.0)
            if self.can_thread.is_alive():
                self.get_logger().warn("CAN thread did not terminate in time")
        
        # Close CAN bus
        if hasattr(self, 'bus') and not self.sim_mode:
            try:
                self.get_logger().info("Closing CAN bus...")
                self.bus.shutdown()
            except Exception as e:
                self.get_logger().error(f"Error closing CAN bus: {e}")
                
        self.get_logger().info("Wheel tracker node shut down completed")
        self._shutdown_complete = True

def main(args=None):
    # Check command-line arguments for --sim flag
    sim_mode = '--sim' in sys.argv
    
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create the node
    wheel_tracker = None
    
    try:
        # Create and initialize the node
        wheel_tracker = WheelTrackerNode(sim_mode=sim_mode)
        
        # Spin the node
        rclpy.spin(wheel_tracker)
    except KeyboardInterrupt:
        # This should be caught by our signal handler, but just in case
        if wheel_tracker:
            wheel_tracker.get_logger().info("Keyboard interrupt received, shutting down.")
    except Exception as e:
        print(f"Error in wheel tracker node: {e}")
    finally:
        # Ensure proper shutdown
        if wheel_tracker:
            wheel_tracker.shutdown()
            wheel_tracker.destroy_node()
        
        # Ensure ROS is shut down
        if rclpy.ok():
            rclpy.shutdown()
            
        print("Wheel tracker node has exited")

if __name__ == '__main__':
    main() 