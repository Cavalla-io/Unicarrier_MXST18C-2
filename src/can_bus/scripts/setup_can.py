#!/usr/bin/env python3

# Standalone script to set up CAN interfaces

import os
import sys
import time
import subprocess

def run_sudo_command(command):
    print(f"Running: sudo {command}")
    return os.system(f"sudo {command}")

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

def create_can_interface(interface):
    """Create a CAN interface if it doesn't exist"""
    # First check if we need to create the interface
    if check_interface_exists(interface):
        print(f"Interface {interface} already exists")
    else:
        print(f"Creating {interface} interface")
        # Create the interface as a virtual CAN device
        run_sudo_command(f"ip link add dev {interface} type vcan")

def setup_can_interface(interface):
    """Set up a CAN interface properly"""
    print(f"\nSetting up {interface}...")
    
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

def main():
    print("Setting up CAN interfaces...")
    
    # Set up CAN buffer sizes
    run_sudo_command("sysctl -w net.core.rmem_max=4194304")
    run_sudo_command("sysctl -w net.core.wmem_max=4194304")
    run_sudo_command("sysctl -w net.core.rmem_default=2097152")
    run_sudo_command("sysctl -w net.core.wmem_default=2097152")
    
    # Set up each interface
    setup_can_interface("can2")
    setup_can_interface("can3")
    
    # Check status
    print("\nChecking interface status:")
    os.system("ip -d link show can2 || echo 'can2 not available'")
    os.system("ip -d link show can3 || echo 'can3 not available'")
    
    print("\nCAN interfaces setup complete!")

if __name__ == "__main__":
    main() 