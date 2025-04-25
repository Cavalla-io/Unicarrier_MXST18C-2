import os
import can
import argparse
import threading

# Initialize CAN buses
can0 = can.interface.Bus(channel='can0', bustype='socketcan')
can1 = can.interface.Bus(channel='can1', bustype='socketcan')

# Global state
blocked_ids = set()  # Set of blocked IDs
blocked_ids_lock = threading.Lock()  # Lock to protect access to blocked_ids
passthrough_active = True  # Flag to control passthrough state


def run_passthrough():
    """
    Continuously pass messages between can0 and can1 with optional filtering.
    """
    while passthrough_active:
        try:
            # Non-blocking receive from CAN0
            can0_msg = can0.recv(timeout=0)
            if can0_msg:
                with blocked_ids_lock:
                    if can0_msg.arbitration_id not in blocked_ids:
                        can1.send(can0_msg)  # Forward to CAN1

            # Non-blocking receive from CAN1
            can1_msg = can1.recv(timeout=0)
            if can1_msg:
                with blocked_ids_lock:
                    if can1_msg.arbitration_id not in blocked_ids:
                        can0.send(can1_msg)  # Forward to CAN0
        except can.CanError as e:
            print(f"CAN passthrough error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")


def update_blocked_ids(new_ids, remove=False):
    """
    Update the blocked IDs dynamically.

    Args:
        new_ids: A set of arbitration IDs to block or unblock.
        remove: If True, unblock the provided IDs. Otherwise, block them.
    """
    global blocked_ids
    with blocked_ids_lock:
        if remove:
            blocked_ids.difference_update(new_ids)
            print(f"Unblocked IDs: {new_ids}")
        else:
            blocked_ids.update(new_ids)
            print(f"Blocked IDs: {new_ids}")
        print(f"Current blocked IDs: {blocked_ids}")


if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser(description="CAN passthrough with filtering.")
    parser.add_argument(
        '--block',
        type=lambda x: int(x, 16),  # Parse hexadecimal values
        nargs='*',
        default=[],
        help="List of CAN message IDs to block initially (e.g., --block 0x123 0x456)."
    )
    parser.add_argument(
        '--update-block',
        type=lambda x: int(x, 16),  # Parse hexadecimal values
        nargs='*',
        help="Update blocked IDs dynamically (e.g., --update-block 0x123 0x456)."
    )
    parser.add_argument(
        '--unblock',
        type=lambda x: int(x, 16),  # Parse hexadecimal values
        nargs='*',
        help="Unblock specific IDs dynamically (e.g., --unblock 0x123 0x456)."
    )
    args = parser.parse_args()

    # If the --update-block or --unblock argument is provided, update IDs and exit
    if args.update_block:
        new_ids = set(args.update_block)
        update_blocked_ids(new_ids)
        exit(0)

    if args.unblock:
        new_ids = set(args.unblock)
        update_blocked_ids(new_ids, remove=True)
        exit(0)

    # Update blocked IDs at the start
    initial_blocked_ids = set(args.block)
    with blocked_ids_lock:
        blocked_ids.update(initial_blocked_ids)

    print(f"Initial blocked IDs: {blocked_ids}")

    # Start the passthrough in a separate thread
    passthrough_thread = threading.Thread(target=run_passthrough)
    passthrough_thread.daemon = True
    passthrough_thread.start()

    try:
        while True:
            # Allow dynamic updates through the terminal
            user_input = input("Enter IDs to block/unblock (comma-separated, hex, e.g., +0x123,+0x456,-0x789), or 'exit': ").strip()
            if user_input.lower() == 'exit':
                passthrough_active = False
                break
            try:
                to_block = {int(x[1:], 16) for x in user_input.split(',') if x.startswith('+')}
                to_unblock = {int(x[1:], 16) for x in user_input.split(',') if x.startswith('-')}
                if to_block:
                    update_blocked_ids(to_block)
                if to_unblock:
                    update_blocked_ids(to_unblock, remove=True)
            except ValueError:
                print("Invalid input. Please enter valid hexadecimal IDs.")
    except KeyboardInterrupt:
        print("\nExiting passthrough...")
        passthrough_active = False

    # Wait for the passthrough thread to finish
    passthrough_thread.join()
    print("Passthrough stopped.")
