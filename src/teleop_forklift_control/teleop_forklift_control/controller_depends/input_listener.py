#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time  # Import time for timestamps
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class InputListener:
    def __init__(self, parent_node):
        self.node = parent_node  # Store a reference to the parent node
        self._lock = threading.Lock()
        # Drive commands: throttle (float), steering direction: 'L', 'R', or None.
        self.drive_command = {"throttle": 0.00, "steering": None}
        # Lift commands: lift, lower, sideshift, drive, and tilt.
        self.lift_command = {"lift": False, "lower": False, "sideshift": None,
                             "drive": "NEUTRAL", "tilt": None, "fast_lower": False}
        
        # Create a QoS profile for the Joy subscription with BEST_EFFORT reliability
        joy_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriptions for Joy and Twist topics.
        self.joy_subscriber = self.node.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            joy_qos)  # Using the custom QoS profile
            
        self.twist_subscriber = self.node.create_subscription(
            Twist,
            '/twist',
            self.twist_callback,
            10)
        # (Optional) A flag if you need to run a loop in a separate thread.
        self.running = True
        
        # Add a counter for throttle logging
        self.throttle_log_counter = 0
        self.log_interval = 20  # Log every 20 joy messages (adjust as needed)
        
        # Add tracking for joy messages
        self.joy_message_count = 0
        self.last_joy_timestamp = time.time()  # Initialize with current time
        self.timeout_threshold = 0.3  # Timeout threshold in seconds
        self.timeout_detected = False  # Flag to track if a timeout has been detected
        
        # Create a timer to check for joy message timeouts
        self.timeout_timer = self.node.create_timer(0.1, self.check_joy_timeout)  # Check every 0.1 seconds
        
        self.node.get_logger().info("Input listener initialized")

    def start(self):
        # No longer need to spin in a separate thread as we're using the parent node
        pass

    def stop(self):
        # Here we don't need to do any extra cleanup for now.
        self.running = False

    def check_joy_timeout(self):
        """Check if it's been too long since the last joy message."""
        current_time = time.time()
        time_since_last_msg = current_time - self.last_joy_timestamp
        
        if time_since_last_msg > self.timeout_threshold:
            # Safety stop: reset all controls to safe/neutral positions
            with self._lock:
                if not self.timeout_detected:
                    # Stop drive commands
                    self.drive_command["steering"] = None
                    self.drive_command["throttle"] = 0.0
                    
                    # Stop lift commands
                    self.lift_command["lift"] = False
                    self.lift_command["lower"] = False
                    self.lift_command["sideshift"] = None
                    self.lift_command["drive"] = "NEUTRAL"
                    self.lift_command["tilt"] = None
                    self.lift_command["fast_lower"] = False
                    
                    self.node.get_logger().warn(f"TIMEOUT SAFETY STOP: No joy messages for {time_since_last_msg:.3f} seconds! All controls stopped.")
                    self.timeout_detected = True
        else:
            # Reset timeout flag if we're receiving messages again
            self.timeout_detected = False

    def joy_callback(self, msg: Joy):
        # Track joy message reception
        self.joy_message_count += 1
        self.last_joy_timestamp = time.time()
        
        # Add debug logging to see the raw message - safely check array lengths first
        # Commenting out excessive logging
        # axis0_val = msg.axes[0] if len(msg.axes) > 0 else "N/A"
        # axis7_val = msg.axes[7] if len(msg.axes) > 7 else "N/A"
        # 
        # # Format the values as strings, with floats formatted to 3 decimal places if they are numbers
        # axis0_str = f"{axis0_val:.3f}" if isinstance(axis0_val, float) else str(axis0_val)
        # axis7_str = f"{axis7_val:.3f}" if isinstance(axis7_val, float) else str(axis7_val)
        # 
        # self.node.get_logger().info(f"Received joy message #{self.joy_message_count}: axes[0]={axis0_str}, axes[7]={axis7_str}, buttons.len={len(msg.buttons)}, axes.len={len(msg.axes)}")
        
        with self._lock:
            # ----- Drive Command Mapping -----
            # For drive throttle, using axis 7
            # Update for joystick that sends values from 0 to 1 instead of -1 to 0
            if len(msg.axes) > 7:
                # Use the value directly if it's in the 0 to 1 range
                throttle_value = msg.axes[7] if msg.axes[7] > 0 else 0
                self.drive_command["throttle"] = throttle_value
                
                # Debug logging occasionally (every 20th message)
                if self.joy_message_count % 20 == 0:
                    self.node.get_logger().info(f"Raw axis[7]={msg.axes[7]:.3f}, mapped throttle={throttle_value:.3f}")
            
            # Log throttle value periodically
            self.throttle_log_counter += 1
            if self.throttle_log_counter >= self.log_interval:
                # self.node.get_logger().info(f"Input Listener - Current throttle value: {throttle_value:.3f}")
                self.throttle_log_counter = 0
            
            # For drive steering, assume the left analog horizontal axis (axes[0]).
            if len(msg.axes) > 0:
                axis_val = msg.axes[0]
                if axis_val < -0.5:
                    self.drive_command["steering"] = 'L'
                elif axis_val > 0.5:
                    self.drive_command["steering"] = 'R'
                else:
                    self.drive_command["steering"] = None

            # ----- Lift Command Mapping -----
            # Check if there are enough buttons in the message.
            # Here we assume:
            # - Button 3 (Y) triggers (forward)
            # - Button 1 (B) triggers (backward)
            if len(msg.buttons) >= 16:  # Make sure we have enough buttons before accessing them
                self.lift_command["lift"] = (msg.buttons[12] == 1)
                self.lift_command["lower"] = (msg.buttons[13] == 1)
                
                # Check button 4 for slow lowering mode
                self.lift_command["fast_lower"] = (msg.buttons[4] == 1)
                
                # Drive command for lift using buttons 1 and 3
                if msg.buttons[1] == 1:
                    self.lift_command["drive"] = "FORWARD"
                elif msg.buttons[3] == 1:
                    self.lift_command["drive"] = "BACKWARD"
                else:
                    self.lift_command["drive"] = "NEUTRAL"
                
                # Sideshift controls
                if msg.buttons[14] == 1:
                    self.lift_command["sideshift"] = 'L'
                elif msg.buttons[15] == 1:
                    self.lift_command["sideshift"] = 'R'
                else:
                    self.lift_command["sideshift"] = None
            else:
                # Log a warning if we don't have enough buttons
                self.node.get_logger().warn(f"Joy message has insufficient buttons: {len(msg.buttons)}/16 required")
                # Reset lift commands to safe values
                self.lift_command["lift"] = False
                self.lift_command["lower"] = False
                self.lift_command["fast_lower"] = False
                self.lift_command["drive"] = "NEUTRAL"
                self.lift_command["sideshift"] = None

    def twist_callback(self, msg: Twist):
        # If you wish to use Twist messages for lift commands, implement your mapping here.
        # For now, we leave it unchanged.
        pass

    def get_drive_command(self):
        with self._lock:
            return self.drive_command.copy()

    def get_lift_command(self):
        with self._lock:
            return self.lift_command.copy()
            
    def get_last_joy_timestamp(self):
        with self._lock:
            return self.last_joy_timestamp

# We no longer need the main() function as this is not a standalone node
