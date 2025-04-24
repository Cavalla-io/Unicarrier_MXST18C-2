#!/usr/bin/env python3

import os
import sys
import time
import subprocess
import argparse
import can  # python-can library

def print_color(text, color):
    """Print colored text"""
    colors = {
        'green': '\033[0;32m',
        'red': '\033[0;31m',
        'yellow': '\033[1;33m',
        'blue': '\033[0;34m',
        'nc': '\033[0m'  # No Color
    }
    print(f"{colors.get(color, colors['nc'])}{text}{colors['nc']}")

def check_can_interfaces():
    """Check if CAN interfaces are properly initialized"""
    print_color("\n=== Checking CAN interfaces ===", "yellow")
    
    interfaces_ok = True
    
    for interface in ['can0', 'can1']:
        # Check if interface exists
        try:
            result = subprocess.run(
                ['ip', 'link', 'show', interface],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            if result.returncode != 0:
                print_color(f"{interface}: NOT FOUND", "red")
                interfaces_ok = False
                continue
                
            # Check if interface is UP
            is_up = 'state UP' in result.stdout
            if is_up:
                print_color(f"{interface}: UP", "green")
            else:
                print_color(f"{interface}: DOWN", "red")
                interfaces_ok = False
            
            # Check bitrate
            if '100000' in result.stdout:
                print_color(f"{interface} bitrate: 100kbps ✓", "green")
            else:
                print_color(f"{interface} bitrate: NOT SET CORRECTLY", "red")
                interfaces_ok = False
                
        except Exception as e:
            print_color(f"Error checking {interface}: {e}", "red")
            interfaces_ok = False
    
    if interfaces_ok:
        print_color("\n✅ CAN interfaces are properly configured", "green")
    else:
        print_color("\n❌ Some issues were found with CAN interfaces", "red")
        print_color("Please run: sudo ../scripts/setup_can_hat.sh", "yellow")
    
    return interfaces_ok

def test_can_communication():
    """Test communication between can0 and can1"""
    print_color("\n=== Testing CAN communication ===", "yellow")
    print_color("Ensure CAN0_H is connected to CAN1_H and CAN0_L to CAN1_L", "yellow")
    
    try:
        # Setup python-can bus interfaces
        bus0 = can.interface.Bus(channel='can0', bustype='socketcan')
        bus1 = can.interface.Bus(channel='can1', bustype='socketcan')
        
        print_color("\nTest 1: Sending message from can0 to can1", "blue")
        # Create and send a message from can0
        msg0 = can.Message(
            arbitration_id=0x123,
            data=[0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False
        )
        
        # Set up a listener for can1
        print("Listening on can1...")
        received0to1 = False
        
        # Send the message on can0
        print(f"Sending on can0: ID=0x123, Data=DEADBEEF")
        bus0.send(msg0)
        
        # Listen for messages on can1 with timeout
        msg_received = bus1.recv(timeout=1.0)
        
        if msg_received and msg_received.arbitration_id == 0x123:
            data_str = ''.join(f'{b:02X}' for b in msg_received.data)
            print(f"Received on can1: ID=0x{msg_received.arbitration_id:X}, Data={data_str}")
            if msg_received.data[:4] == msg0.data[:4]:  # Compare first 4 bytes (DEADBEEF)
                print_color("✅ Message received correctly!", "green")
                received0to1 = True
            else:
                print_color("❌ Message data doesn't match what was sent", "red")
        else:
            print_color("❌ No message received or wrong ID", "red")
        
        print_color("\nTest 2: Sending message from can1 to can0", "blue")
        # Create and send a message from can1
        msg1 = can.Message(
            arbitration_id=0x321,
            data=[0xCA, 0xFE, 0xBA, 0xBE, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False
        )
        
        # Set up a listener for can0
        print("Listening on can0...")
        received1to0 = False
        
        # Send the message on can1
        print(f"Sending on can1: ID=0x321, Data=CAFEBABE")
        bus1.send(msg1)
        
        # Listen for messages on can0 with timeout
        msg_received = bus0.recv(timeout=1.0)
        
        if msg_received and msg_received.arbitration_id == 0x321:
            data_str = ''.join(f'{b:02X}' for b in msg_received.data)
            print(f"Received on can0: ID=0x{msg_received.arbitration_id:X}, Data={data_str}")
            if msg_received.data[:4] == msg1.data[:4]:  # Compare first 4 bytes (CAFEBABE)
                print_color("✅ Message received correctly!", "green")
                received1to0 = True
            else:
                print_color("❌ Message data doesn't match what was sent", "red")
        else:
            print_color("❌ No message received or wrong ID", "red")
        
        # Clean up
        bus0.shutdown()
        bus1.shutdown()
        
        # Print overall results
        if received0to1 and received1to0:
            print_color("\n✅ Bidirectional CAN communication is working correctly!", "green")
            return True
        elif received0to1 or received1to0:
            print_color("\n⚠️ CAN communication working only in one direction", "yellow")
            return False
        else:
            print_color("\n❌ CAN communication failed in both directions", "red")
            print_color("Please check physical connections and termination resistors", "yellow")
            return False
            
    except Exception as e:
        print_color(f"Error during CAN communication test: {e}", "red")
        print_color("Make sure python-can is installed: pip3 install python-can", "yellow")
        return False

def test_with_candump_cansend():
    """Test using candump and cansend utilities"""
    print_color("\n=== Testing with candump and cansend ===", "yellow")
    
    try:
        # First check if can-utils is installed
        result = subprocess.run(['which', 'candump'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode != 0:
            print_color("candump not found. Please install can-utils: sudo apt-get install can-utils", "red")
            return False
        
        # Test can0 -> can1
        print_color("\nTest 1: can0 -> can1 using candump/cansend", "blue")
        
        # Start candump on can1 in the background
        candump_process = subprocess.Popen(
            ['candump', 'can1', '-T', '1000'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Give candump time to start
        time.sleep(0.5)
        
        # Send a message from can0
        test_id = "444"
        test_data = "ABCD1234"
        print(f"Sending message from can0: ID={test_id}, Data={test_data}")
        
        cansend_result = subprocess.run(
            ['cansend', 'can0', f'{test_id}#{test_data}'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        if cansend_result.returncode != 0:
            print_color(f"Failed to send message: {cansend_result.stderr}", "red")
            candump_process.terminate()
            return False
        
        # Wait for the message to be received
        time.sleep(1)
        
        # Check candump output
        candump_process.terminate()
        candump_stdout, candump_stderr = candump_process.communicate(timeout=2)
        
        print(f"candump output: {candump_stdout}")
        
        can1_received = test_id in candump_stdout
        if can1_received:
            print_color("✅ Message received on can1", "green")
        else:
            print_color("❌ Message not received on can1", "red")
        
        # Test can1 -> can0
        print_color("\nTest 2: can1 -> can0 using candump/cansend", "blue")
        
        # Start candump on can0 in the background
        candump_process = subprocess.Popen(
            ['candump', 'can0', '-T', '1000'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Give candump time to start
        time.sleep(0.5)
        
        # Send a message from can1
        test_id = "555"
        test_data = "FEDC4321"
        print(f"Sending message from can1: ID={test_id}, Data={test_data}")
        
        cansend_result = subprocess.run(
            ['cansend', 'can1', f'{test_id}#{test_data}'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        if cansend_result.returncode != 0:
            print_color(f"Failed to send message: {cansend_result.stderr}", "red")
            candump_process.terminate()
            return False
        
        # Wait for the message to be received
        time.sleep(1)
        
        # Check candump output
        candump_process.terminate()
        candump_stdout, candump_stderr = candump_process.communicate(timeout=2)
        
        print(f"candump output: {candump_stdout}")
        
        can0_received = test_id in candump_stdout
        if can0_received:
            print_color("✅ Message received on can0", "green")
        else:
            print_color("❌ Message not received on can0", "red")
        
        # Print overall results
        if can0_received and can1_received:
            print_color("\n✅ Bidirectional CAN communication is working correctly with can-utils!", "green")
            return True
        elif can0_received or can1_received:
            print_color("\n⚠️ CAN communication working only in one direction with can-utils", "yellow")
            return False
        else:
            print_color("\n❌ CAN communication failed in both directions with can-utils", "red")
            print_color("Please check physical connections and termination resistors", "yellow")
            return False
            
    except Exception as e:
        print_color(f"Error during can-utils test: {e}", "red")
        return False

def main():
    parser = argparse.ArgumentParser(description='Test WaveShare 2-CH CAN HAT')
    parser.add_argument('--check-only', action='store_true', help='Only check interfaces, no communication test')
    parser.add_argument('--can-utils', action='store_true', help='Use candump/cansend for testing')
    parser.add_argument('--python-can', action='store_true', help='Use python-can for testing')
    
    args = parser.parse_args()
    
    # If no specific tests are selected, run all
    if not (args.check_only or args.can_utils or args.python_can):
        args.check_only = True
        args.can_utils = True
        args.python_can = True
    
    print_color("===== WaveShare 2-CH CAN HAT Tester =====", "yellow")
    
    # Always check interfaces
    interfaces_ok = check_can_interfaces()
    
    if not interfaces_ok:
        print_color("\nFix interface issues before continuing with tests", "red")
        if not args.check_only:
            print_color("Continuing with tests anyway...", "yellow")
    
    if args.check_only and not (args.can_utils or args.python_can):
        return
    
    print_color("\n>>> Physical connection required for the following tests <<<", "yellow")
    print_color("Make sure you have connected:", "yellow")
    print_color("- CAN0_H to CAN1_H", "yellow") 
    print_color("- CAN0_L to CAN1_L", "yellow")
    
    if args.can_utils:
        test_with_candump_cansend()
    
    if args.python_can:
        test_can_communication()
    
    print_color("\n===== Test Complete =====", "yellow")
    print_color("If tests failed, check:", "blue")
    print_color("1. Physical connections between CAN0 and CAN1", "blue")
    print_color("2. Termination resistors on the HAT", "blue")
    print_color("3. SPI interface configuration", "blue")
    print_color("4. Run 'sudo ../scripts/setup_can_hat.sh' to reconfigure", "blue")

if __name__ == "__main__":
    main() 