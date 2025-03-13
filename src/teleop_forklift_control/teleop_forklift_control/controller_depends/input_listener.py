#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class InputListener:
    def __init__(self, parent_node):
        self.node = parent_node  # Store a reference to the parent node
        self._lock = threading.Lock()
        # Drive commands: throttle (boolean), steering direction: 'L', 'R', or None.
        self.drive_command = {"throttle": False, "steering": None}
        # Lift commands: gas, lift, lower, sideshift, drive, and tilt.
        self.lift_command = {"gas": False, "lift": False, "lower": False, "sideshift": None,
                             "drive": "NEUTRAL", "tilt": None}
        # Create subscriptions for Joy and Twist topics.
        self.joy_subscriber = self.node.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.twist_subscriber = self.node.create_subscription(
            Twist,
            '/twist',
            self.twist_callback,
            10)
        # (Optional) A flag if you need to run a loop in a separate thread.
        self.running = True
        
        self.node.get_logger().info("Input listener initialized")

    def start(self):
        # No longer need to spin in a separate thread as we're using the parent node
        pass

    def stop(self):
        # Here we don't need to do any extra cleanup for now.
        self.running = False

    def joy_callback(self, msg: Joy):
        with self._lock:
            # ----- Drive Command Mapping -----
            # For drive throttle, assume button 0 (A) is used.
            self.drive_command["throttle"] = (msg.buttons[0] == 1) if len(msg.buttons) > 0 else False

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
            # - Button 0 (A) is used for drive throttle only, not for lift gas
            if len(msg.buttons) >= 8:
                self.lift_command["lift"] = (msg.buttons[12] == 1)
                self.lift_command["lower"] = (msg.buttons[13] == 1)
                
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

# We no longer need the main() function as this is not a standalone node
