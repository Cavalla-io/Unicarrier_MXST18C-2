import os
import can
import subprocess

# Function to run a command with sudo
def run_sudo_command(command):
    full_command = f"sudo {command}"
    print(f"Running: {full_command}")
    return os.system(full_command)

# Function to check if an interface exists
def check_interface_exists(interface):
    """Check if the interface exists"""
    try:
        result = subprocess.run(
            ['ip', 'link', 'show', interface],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return result.returncode == 0
    except Exception:
        return False

# Function to create a CAN interface if it doesn't exist
def create_can_interface(interface):
    """Create a CAN interface if it doesn't exist"""
    # First check if we need to create the interface
    if check_interface_exists(interface):
        print(f"Interface {interface} already exists")
    else:
        print(f"Creating {interface} interface")
        # Create the interface as a virtual CAN device
        run_sudo_command(f"ip link add dev {interface} type vcan")

# Function to check the state of a CAN interface
def check_can_state(interface):
    try:
        # Run 'ip link show' to check the interface state
        result = subprocess.run(
            ['ip', 'link', 'show', interface],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        # Look for 'state UP' in the output
        if 'state UP' in result.stdout:
            return True, f"{interface} is UP"
        elif 'state DOWN' in result.stdout:
            return False, f"{interface} is DOWN"
        else:
            return False, f"{interface} state is unknown"
    except Exception as e:
        return False, f"Error checking {interface} state: {e}"

# Function to reset a CAN interface using sudo directly
def reset_can_interface(interface):
    """Set up a CAN interface properly"""
    print(f"Setting up {interface}...")
    
    # Make sure the interface exists before trying to configure it
    create_can_interface(interface)
    
    # Now set it up
    run_sudo_command(f"ip link set {interface} down")
    
    # Try to set it up as CAN, but if that fails, just bring it up
    can_cmd = f"ip link set {interface} up type can bitrate 250000"
    if run_sudo_command(can_cmd) != 0:
        print(f"Warning: Could not set {interface} as CAN type, bringing up without type specification")
        run_sudo_command(f"ip link set {interface} up")
    
    # Set the queue length
    run_sudo_command(f"ip link set {interface} txqueuelen 10000")
    
    print(f"{interface} setup completed")
    return True

# Function to set buffer sizes for CAN interfaces using sudo directly
def set_can_buffers():
    try:
        run_sudo_command("sysctl -w net.core.rmem_max=4194304")
        run_sudo_command("sysctl -w net.core.wmem_max=4194304")
        run_sudo_command("sysctl -w net.core.rmem_default=2097152")
        run_sudo_command("sysctl -w net.core.wmem_default=2097152")
        print("CAN buffer sizes have been set")
    except Exception as e:
        print(f"Error setting CAN buffer sizes: {e}")

# Function to handle CAN startup logic with direct sudo commands
def can_startup():
    # Set buffer sizes for optimal performance
    set_can_buffers()

    # Check the states of can2 and can3
    can2_ready, state_can2 = check_can_state('can2')
    can3_ready, state_can3 = check_can_state('can3')
    
    print(state_can2)
    print(state_can3)
    
    # Set up the interfaces
    reset_can_interface('can2')
    reset_can_interface('can3')
    
    # Return true to indicate success - the node will try to use the interfaces
    return True

# Function to handle CAN shutdown
def can_shutdown():
    print("Shutting down CAN interfaces...")
    # Only attempt to shut down interfaces if they exist
    if check_interface_exists('can2'):
        run_sudo_command(f"ip link set can2 down")
    if check_interface_exists('can3'):
        run_sudo_command(f"ip link set can3 down") 