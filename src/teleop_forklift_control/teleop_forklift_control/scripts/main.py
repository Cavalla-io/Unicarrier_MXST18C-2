#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time

# Import your controller classes
from teleop_forklift_control.controller_depends.input_listener import InputListener
from teleop_forklift_control.controller_depends.drive_controller import DriveController
from teleop_forklift_control.controller_depends.lift_controller import LiftController

class CombinedControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller_node')
        
        # Create the shared input listener as a component within this node
        self.listener = InputListener(self)
        
        # Instantiate your controllers (drive and lift) using the listener.
        self.drive_ctrl = DriveController(self.listener)
        self.lift_ctrl = LiftController(self.listener)
        
        # Create ROS timers to periodically call update functions.
        self.drive_timer = self.create_timer(0.05, self.drive_update_callback)  # ~20 Hz
        self.lift_timer = self.create_timer(1/65, self.lift_update_callback)     # ~65 Hz

        self.get_logger().info("Combined controller node started.")

    def drive_update_callback(self):
        self.drive_ctrl.update()

    def lift_update_callback(self):
        self.lift_ctrl.update()

    def shutdown(self):
        self.get_logger().info("Shutting down combined controller node.")
        self.drive_ctrl.close()
        self.lift_ctrl.close()
        self.listener.stop()

def main(args=None):
    rclpy.init(args=args)
    # Create the node
    combined_node = CombinedControllerNode()
    
    try:
        rclpy.spin(combined_node)
    except KeyboardInterrupt:
        combined_node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        combined_node.shutdown()
        combined_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
