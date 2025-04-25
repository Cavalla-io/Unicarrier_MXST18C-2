import os
import can
import subprocess

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
            return f"{interface} is UP"
        elif 'state DOWN' in result.stdout:
            return f"{interface} is DOWN"
        else:
            return f"{interface} state is unknown"
    except Exception as e:
        return f"Error checking {interface} state: {e}"

# Function to reset a CAN interface
def reset_can_interface(interface):
    os.system(f'sudo ifconfig {interface} down')
    os.system(f'sudo ip link set {interface} up type can bitrate 250000')
    os.system(f'sudo ip link set {interface} txqueuelen 10000')
    print(f"{interface} has been reset")

# Function to create a CAN interface if it doesn't exist
def create_can_interface(interface, device="can0"):
    try:
        # Check if the interface already exists
        result = subprocess.run(
            ['ip', 'link', 'show', interface],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # If return code is 0, interface exists
        if result.returncode == 0:
            print(f"{interface} already exists")
        else:
            # Create virtual CAN interface
            # Note: In a real system, you'd likely link this to physical hardware
            # For can2 and can3, you might need to do more hardware-specific setup
            print(f"Creating {interface} as virtual CAN interface")
            os.system(f'sudo ip link add dev {interface} type vcan')
            
        # Set it up with the right parameters
        reset_can_interface(interface)
        
    except Exception as e:
        print(f"Error creating/setting up {interface}: {e}")

# Function to set buffer sizes for CAN interfaces
def set_can_buffers():
    try:
        os.system('sudo sysctl -w net.core.rmem_max=4194304')
        os.system('sudo sysctl -w net.core.wmem_max=4194304')
        os.system('sudo sysctl -w net.core.rmem_default=2097152')
        os.system('sudo sysctl -w net.core.wmem_default=2097152')
        print("CAN buffer sizes have been set")
    except Exception as e:
        print(f"Error setting CAN buffer sizes: {e}")

# Function to handle CAN startup logic
def can_startup():
    # Ensure buffer sizes are set before starting CAN interfaces
    set_can_buffers()

    # Check the states of can2 and can3
    state_can2 = check_can_state('can2')
    state_can3 = check_can_state('can3')
    
    print(state_can2)
    print(state_can3)
    
    # Create or reset interfaces as needed
    if "is UP" in state_can2:
        print("can2 is UP. Resetting to ensure correct parameters...")
        reset_can_interface('can2')
    elif "is DOWN" in state_can2:
        print("can2 is DOWN. Starting up...")
        reset_can_interface('can2')
    else:
        print("can2 does not exist. Creating...")
        create_can_interface('can2')

    if "is UP" in state_can3:
        print("can3 is UP. Resetting to ensure correct parameters...")
        reset_can_interface('can3')
    elif "is DOWN" in state_can3:
        print("can3 is DOWN. Starting up...")
        reset_can_interface('can3')
    else:
        print("can3 does not exist. Creating...")
        create_can_interface('can3')

#Shutdown logic
def shutdown_can_interface(interface):
    os.system(f'sudo ifconfig {interface} down')

#Function to handle CAN shutdown
def can_shutdown():
    shutdown_can_interface('can2')
    shutdown_can_interface('can3') 