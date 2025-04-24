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

# Function to check the state of a CAN interface
def check_can_state(interface):
    """Check the state of a CAN interface"""
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

# Function to load CAN kernel modules
def load_can_modules():
    """Load necessary CAN kernel modules"""
    print("Loading CAN kernel modules...")
    # Insert the CAN BUS subsystem support module
    run_sudo_command("modprobe can")
    # Insert the raw CAN protocol module
    run_sudo_command("modprobe can_raw")
    # Add real CAN interface support (for Jetson, mttcan)
    run_sudo_command("modprobe mttcan")

# Function to reset a CAN interface using sudo directly
def reset_can_interface(interface, bitrate=250000):
    """Set up a CAN interface properly"""
    print(f"Setting up {interface}...")
    
    # First bring down the interface if it exists
    if check_interface_exists(interface):
        run_sudo_command(f"ip link set {interface} down")
    
    # Set it up as a CAN interface with specified bitrate
    can_cmd = f"ip link set {interface} up type can bitrate {bitrate} berr-reporting on"
    if run_sudo_command(can_cmd) != 0:
        print(f"Warning: Could not set {interface} as CAN type with bitrate {bitrate}")
        return False
    
    # Set the queue length
    run_sudo_command(f"ip link set {interface} txqueuelen 10000")
    
    # Check if the interface is up
    is_up, status = check_can_state(interface)
    print(status)
    
    print(f"{interface} setup completed")
    return is_up

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
    # Load required kernel modules
    load_can_modules()
    
    # Set buffer sizes for optimal performance
    set_can_buffers()

    # Check the states of can0 and can1
    can0_ready, state_can0 = check_can_state('can0')
    can1_ready, state_can1 = check_can_state('can1')
    
    print(state_can0)
    print(state_can1)
    
    # Set up the interfaces
    reset_can_interface('can0')
    reset_can_interface('can1')
    
    # Return true to indicate success - the node will try to use the interfaces
    return True

# Function to handle CAN shutdown
def can_shutdown():
    print("Shutting down CAN interfaces...")
    # Only attempt to shut down interfaces if they exist
    if check_interface_exists('can0'):
        run_sudo_command(f"ip link set can0 down")
    if check_interface_exists('can1'):
        run_sudo_command(f"ip link set can1 down") 