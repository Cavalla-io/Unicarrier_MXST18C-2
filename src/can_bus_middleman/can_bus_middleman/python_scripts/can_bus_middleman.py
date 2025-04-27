import os
import socket
import can
import threading
import can_bus_setup
from can_bus_lift_msg import LiftController

# Configure CAN interfaces
CAN0_INTERFACE = 'can0'
CAN1_INTERFACE = 'can1'

bus_can0 = can.interface.Bus(channel=CAN0_INTERFACE, bustype='socketcan')
bus_can1 = can.interface.Bus(channel=CAN1_INTERFACE, bustype='socketcan')

# Set up a TCP server
HOST = '127.0.0.1'  # localhost
PORT = 65432        # Port to listen on

# Initialize the lift controller
lift_controller = LiftController(bus_can1, 0x1A0)

# Global flags and state
server_running = True
passthrough_active = True
blocked_ids = set()  # Set of blocked IDs
blocked_ids_lock = threading.Lock()  # Lock to protect access to blocked_ids

def handle_client_connection(conn, addr):
    """
    Handle incoming client connections.
    """
    global passthrough_active
    print(f"Connected by {addr}")
    while server_running:
        try:
            data = conn.recv(1024)  # Receive command
            if not data:
                break

            command = data.decode().strip()
            print(f"Received command: {command}")

            if command == "LIFT_UP":
                passthrough_active = False
                stop_other_operations()
                with blocked_ids_lock:
                    blocked_ids.update({0x1A0})  # Block IDs during lifting
                lift_controller.start_lift("UP")
                with blocked_ids_lock:
                    blocked_ids.difference_update({0x1A0})  # Unblock IDs after lifting
                passthrough_active = True
            elif command == "LIFT_DOWN":
                passthrough_active = False
                stop_other_operations()
                with blocked_ids_lock:
                    blocked_ids.update({0x1A0})  # Block IDs during lifting
                lift_controller.start_lift("DOWN")
                with blocked_ids_lock:
                    blocked_ids.difference_update({0x1A0})  # Unblock IDs after lifting
                passthrough_active = True
            elif command == "STOP":
                lift_controller.stop_lift()
                passthrough_active = True
            elif command == "STATUS_CHECK":
                passthrough_active = False
                send_can_message(bus_can0, 0x200, [0x00])  # Example status check message
                passthrough_active = True
            else:
                print(f"Unknown command: {command}")
        except Exception as e:
            print(f"Error handling client {addr}: {e}")
            break
    conn.close()


def stop_other_operations():
    """
    Stop any ongoing operations before starting a new one.
    """
    lift_controller.stop_lift()


def send_can_message(bus, arbitration_id, data):
    """
    Send a CAN message on the specified bus.
    """
    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}: {msg}")
    except can.CanError as e:
        print(f"Failed to send message on {bus.channel_info}: {e}")


def passthrough_loop():
    """
    Continuously pass messages between CAN0 and CAN1 when passthrough is active,
    with filtering for blocked IDs.
    """
    while server_running:
        if passthrough_active:
            try:
                # Non-blocking receive from CAN0
                can0_msg = bus_can0.recv(timeout=0)
                if can0_msg:
                    with blocked_ids_lock:
                        if can0_msg.arbitration_id not in blocked_ids:
                            bus_can1.send(can0_msg)  # Forward to CAN1

                # Non-blocking receive from CAN1
                can1_msg = bus_can1.recv(timeout=0)
                if can1_msg:
                    with blocked_ids_lock:
                        if can1_msg.arbitration_id not in blocked_ids:
                            bus_can0.send(can1_msg)  # Forward to CAN0
            except can.CanError as e:
                print(f"CAN passthrough error: {e}")
        else:
            # Sleep briefly to reduce CPU usage when passthrough is inactive
            threading.Event().wait(0.01)


def start_server():
    """
    Start the TCP server to listen for commands.
    """
    global server_running
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reuse of address
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        print(f"Server listening on {HOST}:{PORT}")

        while server_running:
            try:
                conn, addr = server_socket.accept()
                client_thread = threading.Thread(target=handle_client_connection, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
            except KeyboardInterrupt:
                print("\nServer interrupted by user.")
                break


if __name__ == "__main__":
    try:
        # Ensure CAN bus setup is performed
        can_bus_setup.can_startup()

        # Start the passthrough loop in a separate thread
        passthrough_thread = threading.Thread(target=passthrough_loop)
        passthrough_thread.daemon = True
        passthrough_thread.start()

        # Start the server
        start_server()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        server_running = False  # Stop the server gracefully
        lift_controller.stop_lift()
        can_bus_setup.can_shutdown()
        print("Server and CAN interfaces shut down successfully.")
