#!/usr/bin/env python3

# Standalone script to set up physical CAN interfaces on Jetson Orin AGX

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

def check_can_state(interface):
    """Check the state of a CAN interface"""
    try:
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

def check_pinmux_configuration():
    """Check if the pinmux is correctly configured for CAN interfaces"""
    # Install busybox if needed
    try:
        # Check if busybox is installed
        result = subprocess.run(
            ['which', 'busybox'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        if result.returncode != 0:
            print("Installing busybox for pinmux configuration...")
            run_sudo_command("apt-get update")
            run_sudo_command("apt-get install -y busybox")
    except Exception as e:
        print(f"Error checking busybox installation: {e}")
        return False
    
    # Define the correct pinmux values for Jetson Orin AGX
    pinmux_config = {
        "can0_din": {"addr": "0x0c303018", "value": "0xc458"},
        "can0_dout": {"addr": "0x0c303010", "value": "0xc400"},
        "can1_din": {"addr": "0x0c303008", "value": "0xc458"},
        "can1_dout": {"addr": "0x0c303000", "value": "0xc400"}
    }
    
    # Check current pinmux values
    print("\nChecking pinmux configuration...")
    pinmux_correct = True
    
    for pin, config in pinmux_config.items():
        try:
            # Read current value
            result = subprocess.run(
                ["sudo", "busybox", "devmem", config["addr"]],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            current_value = result.stdout.strip()
            expected_value = config["value"].replace("0x", "")
            
            # Compare with expected value (ignore 0x prefix)
            if expected_value.lower() not in current_value.lower():
                print(f"{pin}: INCORRECT - Current: {current_value}, Expected: {config['value']}")
                pinmux_correct = False
            else:
                print(f"{pin}: CORRECT - {current_value}")
        except Exception as e:
            print(f"Error checking {pin} pinmux: {e}")
            pinmux_correct = False
    
    return pinmux_correct

def configure_pinmux():
    """Configure the pinmux for CAN interfaces on Jetson Orin AGX"""
    print("\nConfiguring pinmux for CAN interfaces...")
    
    # Define the correct pinmux values
    pinmux_config = {
        "can0_din": {"addr": "0x0c303018", "value": "0xc458"},
        "can0_dout": {"addr": "0x0c303010", "value": "0xc400"},
        "can1_din": {"addr": "0x0c303008", "value": "0xc458"},
        "can1_dout": {"addr": "0x0c303000", "value": "0xc400"}
    }
    
    # Set each pinmux register
    success = True
    for pin, config in pinmux_config.items():
        try:
            cmd = f"busybox devmem {config['addr']} w {config['value']}"
            result = run_sudo_command(cmd)
            if result != 0:
                print(f"Failed to configure {pin}")
                success = False
            else:
                print(f"Successfully configured {pin}")
        except Exception as e:
            print(f"Error configuring {pin}: {e}")
            success = False
    
    return success

def load_can_modules():
    """Load necessary CAN kernel modules"""
    print("Loading CAN kernel modules...")
    # Insert the CAN BUS subsystem support module
    run_sudo_command("modprobe can")
    # Insert the raw CAN protocol module (CAN-ID filtering)
    run_sudo_command("modprobe can_raw")
    # Add real CAN interface support (for Jetson, mttcan)
    run_sudo_command("modprobe mttcan")

def setup_can_interface(interface, bitrate=250000):
    """Set up a physical CAN interface properly"""
    print(f"\nSetting up {interface}...")
    
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

def main():
    print("Setting up CAN interfaces for Jetson Orin AGX...")
    
    # Check and configure pinmux settings
    pinmux_correct = check_pinmux_configuration()
    if not pinmux_correct:
        print("\nPinmux is not correctly configured for CAN interfaces.")
        print("Attempting to configure pinmux...")
        if configure_pinmux():
            print("✅ Pinmux configuration successful")
        else:
            print("❌ Failed to configure pinmux. Please configure manually:")
            print("   sudo busybox devmem 0x0c303018 w 0xc458  # can0_din")
            print("   sudo busybox devmem 0x0c303010 w 0xc400  # can0_dout")
            print("   sudo busybox devmem 0x0c303008 w 0xc458  # can1_din")
            print("   sudo busybox devmem 0x0c303000 w 0xc400  # can1_dout")
    else:
        print("✅ Pinmux already correctly configured for CAN interfaces")
    
    # Load required kernel modules
    load_can_modules()
    
    # Set up CAN buffer sizes
    run_sudo_command("sysctl -w net.core.rmem_max=4194304")
    run_sudo_command("sysctl -w net.core.wmem_max=4194304")
    run_sudo_command("sysctl -w net.core.rmem_default=2097152")
    run_sudo_command("sysctl -w net.core.wmem_default=2097152")
    
    # Set up each physical interface - can0 and can1 for Jetson Orin AGX
    setup_can_interface("can0")
    setup_can_interface("can1")
    
    # Check status
    print("\nChecking interface status:")
    os.system("ip -d link show can0 || echo 'can0 not available'")
    os.system("ip -d link show can1 || echo 'can1 not available'")
    
    print("\nCAN interfaces setup complete!")
    print("Note: Make sure you have connected CAN transceivers to the Jetson Orin AGX pins:")
    print("CAN0_DIN: pin 29, CAN0_DOUT: pin 31, CAN1_DIN: pin 37, CAN1_DOUT: pin 33 on the 40-pin header (J30)")
    
    # Suggest running the test script
    print("\nYou can verify the setup with:")
    print("./test_can_bus.sh --test-physical")

if __name__ == "__main__":
    main() 