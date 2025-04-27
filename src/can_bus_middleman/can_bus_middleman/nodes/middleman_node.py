import os
import can
import threading
import time
import argparse
from can_bus_middleman.python_scripts import can_bus_setup

# Add ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CanBusMiddleman(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('can_bus_middleman')
        
        # Create subscription to command topic
        self.command_subscription = self.create_subscription(
            String,
            '/can_middleman/command',
            self.command_callback,
            10  # QoS profile depth
        )
        self.get_logger().info('Subscribed to /can_middleman/command topic')
        
        # Configuration
        self.CAN0_INTERFACE = 'can0'
        self.CAN1_INTERFACE = 'can1'
        
        # Initialize buses after setup
        self.bus_can0 = None
        self.bus_can1 = None
        
        # Control flags
        self.running = True
        self.passthrough_active = True
        self.blocked_ids = set()  # Set of blocked IDs
        self.blocked_ids_lock = threading.Lock()  # Lock to protect access to blocked_ids
        
        # Threads
        self.passthrough_thread = None
        self.ros_spin_thread = None

    def command_callback(self, msg):
        """Process commands received from ROS 2 topic"""
        command = msg.data.strip()
        self.get_logger().info(f'Received command: {command}')
        
        try:
            if command.startswith("block "):
                id_str = command.split(" ")[1]
                self.block_id(id_str)
            elif command.startswith("unblock "):
                id_str = command.split(" ")[1]
                self.unblock_id(id_str)
            elif command == "list":
                with self.blocked_ids_lock:
                    if not self.blocked_ids:
                        self.get_logger().info("No IDs are currently blocked")
                    else:
                        blocked_list = ", ".join([f"0x{id:X}" for id in self.blocked_ids])
                        self.get_logger().info(f"Blocked IDs: {blocked_list}")
            elif command == "status":
                status = self.get_system_status()
                self.get_logger().info(f"System status:\n{status}")
            elif command == "pause":
                self.passthrough_active = False
                self.get_logger().info("CAN passthrough paused")
            elif command == "resume":
                self.passthrough_active = True
                self.get_logger().info("CAN passthrough resumed")
            else:
                self.get_logger().warn(f"Unknown command: {command}")
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")

    def setup_can_interfaces(self):
        """Initialize CAN interfaces"""
        self.get_logger().info("Setting up CAN interfaces...")
        try:
            # Call the setup function to ensure CAN interfaces are ready
            can_bus_setup.can_startup()
            
            # Now initialize the buses
            self.bus_can0 = can.interface.Bus(channel=self.CAN0_INTERFACE, bustype='socketcan')
            self.bus_can1 = can.interface.Bus(channel=self.CAN1_INTERFACE, bustype='socketcan')
            
            self.get_logger().info("CAN interfaces successfully initialized")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN interfaces: {e}")
            return False

    def block_id(self, can_id):
        """Block a specific CAN ID from passing through"""
        try:
            id_hex = int(can_id, 16) if isinstance(can_id, str) else int(can_id)
            with self.blocked_ids_lock:
                self.blocked_ids.add(id_hex)
            self.get_logger().info(f"Blocked CAN ID: 0x{id_hex:X}")
        except ValueError:
            self.get_logger().error(f"Invalid CAN ID format: {can_id}. Use hexadecimal (0xXXX) or decimal format.")

    def unblock_id(self, can_id):
        """Unblock a specific CAN ID"""
        try:
            id_hex = int(can_id, 16) if isinstance(can_id, str) else int(can_id)
            with self.blocked_ids_lock:
                if id_hex in self.blocked_ids:
                    self.blocked_ids.remove(id_hex)
                    self.get_logger().info(f"Unblocked CAN ID: 0x{id_hex:X}")
                else:
                    self.get_logger().info(f"CAN ID 0x{id_hex:X} was not blocked")
        except ValueError:
            self.get_logger().error(f"Invalid CAN ID format: {can_id}. Use hexadecimal (0xXXX) or decimal format.")

    def get_system_status(self):
        """Return the current system status"""
        with self.blocked_ids_lock:
            if not self.blocked_ids:
                blocked_list = "None"
            else:
                blocked_list = ", ".join([f"0x{id:X}" for id in self.blocked_ids])
        
        status = (
            f"CAN Passthrough Active: {self.passthrough_active}\n"
            f"Blocked IDs: {blocked_list}\n"
            f"CAN0 Status: {can_bus_setup.check_can_state('can0')}\n"
            f"CAN1 Status: {can_bus_setup.check_can_state('can1')}"
        )
        return status

    def passthrough_loop(self):
        """
        Continuously pass messages between CAN0 and CAN1 when passthrough is active,
        with filtering for blocked IDs.
        """
        while self.running:
            if self.passthrough_active:
                try:
                    # Non-blocking receive from CAN0
                    can0_msg = self.bus_can0.recv(timeout=0)
                    if can0_msg:
                        with self.blocked_ids_lock:
                            if can0_msg.arbitration_id not in self.blocked_ids:
                                self.bus_can1.send(can0_msg)  # Forward to CAN1

                    # Non-blocking receive from CAN1
                    can1_msg = self.bus_can1.recv(timeout=0)
                    if can1_msg:
                        with self.blocked_ids_lock:
                            if can1_msg.arbitration_id not in self.blocked_ids:
                                self.bus_can0.send(can1_msg)  # Forward to CAN0
                except can.CanError as e:
                    self.get_logger().error(f"CAN passthrough error: {e}")
            else:
                # Sleep briefly to reduce CPU usage when passthrough is inactive
                threading.Event().wait(0.01)

    def cli_interface(self):
        """Simple CLI interface for direct interaction"""
        print("\nCAN Bus Middleman CLI")
        print("Available commands:")
        print("  block <id>     - Block a CAN ID (e.g., block 0x1A0)")
        print("  unblock <id>   - Unblock a CAN ID (e.g., unblock 0x1A0)")
        print("  list           - List currently blocked IDs")
        print("  status         - Show system status")
        print("  pause          - Pause the passthrough")
        print("  resume         - Resume the passthrough")
        print("  quit           - Exit the program")
        
        while self.running:
            try:
                command = input("\nEnter command: ").strip()
                
                if command.startswith("block "):
                    id_str = command.split(" ")[1]
                    self.block_id(id_str)
                elif command.startswith("unblock "):
                    id_str = command.split(" ")[1]
                    self.unblock_id(id_str)
                elif command == "list":
                    with self.blocked_ids_lock:
                        if not self.blocked_ids:
                            print("No IDs are currently blocked")
                        else:
                            for id in self.blocked_ids:
                                print(f"Blocked: 0x{id:X}")
                elif command == "status":
                    print(self.get_system_status())
                elif command == "pause":
                    self.passthrough_active = False
                    print("CAN passthrough paused")
                elif command == "resume":
                    self.passthrough_active = True
                    print("CAN passthrough resumed")
                elif command == "quit" or command == "exit":
                    print("Shutting down...")
                    self.shutdown()
                    break
                else:
                    print(f"Unknown command: {command}")
            except KeyboardInterrupt:
                print("\nShutting down...")
                self.shutdown()
                break
            except Exception as e:
                print(f"Error processing command: {e}")

    def ros_spin(self):
        """Thread function to spin the ROS node"""
        while self.running:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.01)  # Small sleep to prevent CPU overuse

    def start(self):
        """Start all components of the system"""
        if not self.setup_can_interfaces():
            self.get_logger().error("Failed to start due to CAN interface setup issues")
            return False

        # Start the passthrough loop in a separate thread
        self.passthrough_thread = threading.Thread(target=self.passthrough_loop)
        self.passthrough_thread.daemon = True
        self.passthrough_thread.start()
        
        # Start the ROS spin thread
        self.ros_spin_thread = threading.Thread(target=self.ros_spin)
        self.ros_spin_thread.daemon = True
        self.ros_spin_thread.start()
        
        self.get_logger().info("CAN bus middleman started successfully")
        
        # Start the CLI interface in the main thread
        try:
            self.cli_interface()
        except KeyboardInterrupt:
            self.get_logger().info("Program interrupted by user")
            self.shutdown()
            
        return True

    def shutdown(self):
        """Clean shutdown of all components"""
        self.get_logger().info("Initiating shutdown sequence...")
        self.running = False
        
        # Shutdown CAN interfaces
        try:
            can_bus_setup.can_shutdown()
            self.get_logger().info("CAN interfaces shut down")
        except Exception as e:
            self.get_logger().error(f"Error shutting down CAN interfaces: {e}")
        
        self.get_logger().info("Shutdown complete")


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    middleman = CanBusMiddleman()
    
    try:
        middleman.start()
    except Exception as e:
        middleman.get_logger().error(f"Error in main: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
