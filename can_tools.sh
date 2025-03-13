#!/bin/bash

# can_tools.sh - User-friendly interface for CAN bus operations without requiring root
# After running setup_network_privileges.sh, this script should work for regular users

# Set CAN interface and parameters
CAN_INTERFACE="can0"
CAN_BITRATE=250000
CAN_TXQUEUELEN=10000

# Function to check if CAN interface is up
check_can_status() {
    if ip link show ${CAN_INTERFACE} &>/dev/null; then
        STATUS=$(ip -details link show ${CAN_INTERFACE} | grep -oi "UP" || echo "DOWN")
        if [[ "$STATUS" == "UP" ]]; then
            echo "CAN interface ${CAN_INTERFACE} is UP and running."
            BITRATE=$(ip -details link show ${CAN_INTERFACE} | grep -o "bitrate [0-9]*" | awk '{print $2}')
            echo "Bitrate: ${BITRATE:-Unknown} bps"
            TXQLEN=$(ip -details link show ${CAN_INTERFACE} | grep -o "txqueuelen [0-9]*" | awk '{print $2}')
            echo "TX Queue Length: ${TXQLEN:-Unknown}"
            return 0
        else
            echo "CAN interface ${CAN_INTERFACE} exists but is DOWN."
            return 1
        fi
    else
        echo "CAN interface ${CAN_INTERFACE} does not exist or is not accessible."
        return 2
    fi
}

# Function to start CAN interface
start_can() {
    echo "Starting CAN interface ${CAN_INTERFACE}..."
    
    # Try to use our existing permissions first
    if ! ip link show ${CAN_INTERFACE} &>/dev/null; then
        echo "Creating CAN interface..."
        if ! sudo ip link add dev ${CAN_INTERFACE} type can; then
            echo "Failed to create CAN interface. You might need root privileges."
            return 1
        fi
    else
        echo "Taking down existing ${CAN_INTERFACE} interface..."
        sudo ip link set ${CAN_INTERFACE} down
    fi
    
    echo "Configuring ${CAN_INTERFACE} with bitrate ${CAN_BITRATE}..."
    sudo ip link set ${CAN_INTERFACE} type can bitrate ${CAN_BITRATE}
    
    echo "Setting txqueuelen to ${CAN_TXQUEUELEN}..."
    sudo ip link set ${CAN_INTERFACE} txqueuelen ${CAN_TXQUEUELEN}
    
    echo "Bringing up ${CAN_INTERFACE}..."
    sudo ip link set ${CAN_INTERFACE} up
    
    check_can_status
}

# Function to stop CAN interface
stop_can() {
    echo "Stopping CAN interface ${CAN_INTERFACE}..."
    if ip link set ${CAN_INTERFACE} down 2>/dev/null; then
        echo "CAN interface stopped successfully."
    else
        echo "Using sudo to stop CAN interface..."
        sudo ip link set ${CAN_INTERFACE} down
    fi
}

# Function to monitor CAN traffic
monitor_can() {
    if ! command -v candump &>/dev/null; then
        echo "candump not found. Please install can-utils package."
        echo "  sudo apt-get install can-utils"
        return 1
    fi
    
    echo "Monitoring CAN traffic on ${CAN_INTERFACE}..."
    echo "Press Ctrl+C to stop monitoring."
    candump ${CAN_INTERFACE}
}

# Function to send a CAN frame
send_can_frame() {
    if ! command -v cansend &>/dev/null; then
        echo "cansend not found. Please install can-utils package."
        echo "  sudo apt-get install can-utils"
        return 1
    fi
    
    if [ -z "$1" ]; then
        echo "Error: CAN frame not specified."
        echo "Usage: $0 send <frame-in-format-123#DEADBEEF>"
        return 1
    fi
    
    echo "Sending CAN frame: $1"
    cansend ${CAN_INTERFACE} "$1"
    echo "Frame sent."
}

# Function to install CAN utilities if needed
install_can_utils() {
    if ! command -v candump &>/dev/null || ! command -v cansend &>/dev/null; then
        echo "Installing can-utils package..."
        sudo apt-get update && sudo apt-get install -y can-utils
        echo "can-utils installed successfully."
    else
        echo "can-utils already installed."
    fi
}

# Main script logic
case "$1" in
    status)
        check_can_status
        ;;
    start)
        start_can
        ;;
    stop)
        stop_can
        ;;
    restart)
        stop_can
        sleep 1
        start_can
        ;;
    monitor)
        check_can_status >/dev/null || start_can
        monitor_can
        ;;
    send)
        check_can_status >/dev/null || start_can
        send_can_frame "$2"
        ;;
    install)
        install_can_utils
        ;;
    *)
        echo "CAN Bus Tools - User-friendly CAN operations"
        echo "Usage: $0 [command]"
        echo ""
        echo "Commands:"
        echo "  status    Check status of CAN interface"
        echo "  start     Start CAN interface"
        echo "  stop      Stop CAN interface"
        echo "  restart   Restart CAN interface"
        echo "  monitor   Monitor CAN traffic (requires can-utils)"
        echo "  send ID#DATA    Send a CAN frame (e.g., send 123#DEADBEEF)"
        echo "  install   Install CAN utilities (can-utils package)"
        echo ""
        check_can_status
        ;;
esac

exit 0 