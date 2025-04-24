#!/bin/bash
# Comprehensive setup script for WaveShare 2-CH CAN HAT on Jetson Orin AGX
# This script handles installation, configuration, and testing

# Text colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}========== WaveShare 2-CH CAN HAT Setup ===========${NC}"
echo "This script will set up your WaveShare 2-CH CAN HAT with the Jetson Orin AGX"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run as root (sudo).${NC}"
    exit 1
fi

# Function to check if a command was successful
check_status() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Success${NC}"
    else
        echo -e "${RED}✗ Failed${NC}"
        if [ "$1" == "critical" ]; then
            echo -e "${RED}Critical error. Exiting.${NC}"
            exit 1
        fi
    fi
}

echo -e "\n${YELLOW}Step 1: Installing required packages${NC}"
apt-get update
check_status
apt-get install -y can-utils
check_status

echo -e "\n${YELLOW}Step 2: Loading and configuring kernel modules${NC}"
echo "Loading CAN modules..."
modprobe can
check_status
modprobe can_raw
check_status
modprobe can_dev
check_status
modprobe mcp251x
check_status

# Create modules-load.d file for auto-loading at boot
echo "Setting up modules to load at boot..."
cat > /etc/modules-load.d/can-modules.conf << EOF
can
can_raw
can_dev
mcp251x
EOF
check_status

echo -e "\n${YELLOW}Step 3: Checking SPI interface${NC}"
if ls -la /dev/spi* &>/dev/null; then
    echo -e "${GREEN}SPI interface is already enabled.${NC}"
else
    echo -e "${YELLOW}SPI interface not detected. Enabling SPI...${NC}"
    echo "To enable SPI, you need to run: sudo /opt/nvidia/jetson-io/jetson-io.py"
    echo "- Select 'Configure Jetson Expansion Header pins'"
    echo "- Enable SPI"
    echo "- Save and reboot"
    echo -e "${YELLOW}Do you want to run jetson-io tool now? (y/n)${NC}"
    read -r response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        /opt/nvidia/jetson-io/jetson-io.py
        echo -e "${YELLOW}Please reboot and run this script again.${NC}"
        exit 0
    else
        echo -e "${YELLOW}Please enable SPI manually and run this script again.${NC}"
        exit 0
    fi
fi

echo -e "\n${YELLOW}Step 4: Configuring CAN interfaces${NC}"
# Bring down interfaces if they exist
if ip link show can0 &>/dev/null; then
    echo "Bringing down can0..."
    ip link set can0 down
fi
if ip link show can1 &>/dev/null; then
    echo "Bringing down can1..."
    ip link set can1 down
fi

# Configure SPI CAN interfaces
echo "Setting up can0 with 100kbps bitrate..."
ip link set can0 type can bitrate 100000
check_status "critical"

echo "Setting up can1 with 100kbps bitrate..."
ip link set can1 type can bitrate 100000
check_status "critical"

# Bring up interfaces
echo "Bringing up can0..."
ip link set can0 up
check_status "critical"

echo "Bringing up can1..."
ip link set can1 up
check_status "critical"

# Set queue length
echo "Setting queue length..."
ifconfig can0 txqueuelen 65536
ifconfig can1 txqueuelen 65536
check_status

echo -e "\n${YELLOW}Step 5: Creating startup service${NC}"
cat > /etc/systemd/system/can-hat.service << EOF
[Unit]
Description=Setup CAN HAT interfaces
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup_can_hat.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
check_status

# Create the startup script
cat > /usr/local/bin/setup_can_hat.sh << EOF
#!/bin/bash

# Bring down interfaces if they exist
if ip link show can0 &>/dev/null; then
    sudo ip link set can0 down
fi
if ip link show can1 &>/dev/null; then
    sudo ip link set can1 down
fi

# Configure SPI CAN interfaces
sudo ip link set can0 type can bitrate 100000
sudo ip link set can1 type can bitrate 100000

# Bring up interfaces
sudo ip link set can0 up
sudo ip link set can1 up

# Set queue length
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can1 txqueuelen 65536

echo "CAN interfaces configured successfully"
EOF
check_status

# Make the script executable
chmod +x /usr/local/bin/setup_can_hat.sh
check_status

# Enable the service
echo "Enabling and starting CAN HAT service..."
systemctl enable can-hat.service
systemctl start can-hat.service
check_status

echo -e "\n${YELLOW}Step 6: Testing the configuration${NC}"
echo "Checking CAN interface status:"
ip -d link show can0
ip -d link show can1

echo -e "\n${GREEN}Setup completed!${NC}"
echo -e "Current CAN interfaces status:"
ifconfig can0
ifconfig can1

echo -e "\n${YELLOW}===== Physical Connection Instructions =====${NC}"
echo "For testing, physically connect:"
echo "- CAN0_H to CAN1_H"
echo "- CAN0_L to CAN1_L"
echo "Make sure the termination resistors are enabled on the HAT."

echo -e "\n${YELLOW}===== Testing Instructions =====${NC}"
echo "To test communication between CAN interfaces, open two terminals:"
echo "Terminal 1: candump can0"
echo "Terminal 2: cansend can1 123#DEADBEEF"

echo -e "\n${YELLOW}===== ROS Integration =====${NC}"
echo "To update your ROS node to use the HAT, modify can_bus_ros_node.py"
echo "to use 'socketcan' interface instead of direct hardware access."
echo "We recommend using the existing test_can_bus.py for testing first."
echo -e "${GREEN}Done!${NC}" 