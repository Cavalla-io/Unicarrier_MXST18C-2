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

    # Check the states of can0 and can1
    state_can0 = check_can_state('can0')
    state_can1 = check_can_state('can1')
    
    print(state_can0)
    print(state_can1)
    
    # If either interface is up, reset it
    if "is UP" in state_can0:
        print("can0 is still UP. Resetting...")
        reset_can_interface('can0')
    else:
        print("can0 is DOWN. Starting up now...")
        reset_can_interface('can0')

    if "is UP" in state_can1:
        print("can1 is still UP. Resetting...")
        reset_can_interface('can1')
    else:
        print("can1 is DOWN. Starting up now...")
        reset_can_interface('can1')

#Shutdown logic
def shutdown_can_interface(interface):
    os.system(f'sudo ifconfig {interface} down')

#Function to handle CAN shutdown
def can_shutdown():
    shutdown_can_interface('can0')
    shutdown_can_interface('can1')

# Print the results
# print(state_can0)
# print(state_can1)
