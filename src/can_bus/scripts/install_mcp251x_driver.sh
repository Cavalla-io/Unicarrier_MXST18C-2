#!/bin/bash
# MCP2515 Device Tree Setup for WaveShare 2-CH CAN HAT on Jetson Orin

# Text colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}===== WaveShare 2-CH CAN HAT Setup for Jetson Orin =====${NC}"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run as root (sudo).${NC}"
    exit 1
fi

# Install required packages
echo -e "\n${YELLOW}Installing required packages...${NC}"
apt-get update
apt-get install -y can-utils

# Create device tree overlay
echo -e "\n${YELLOW}Creating device tree overlay for MCP2515...${NC}"
mkdir -p /boot/overlays

cat > /boot/overlays/mcp2515-overlay.dts << EOF
/dts-v1/;
/plugin/;

/ {
    compatible = "nvidia,tegra234";
    
    /* Create clock for MCP2515 */
    fragment@0 {
        target-path = "/";
        __overlay__ {
            can_clock: can_clock {
                compatible = "fixed-clock";
                #clock-cells = <0>;
                clock-frequency = <16000000>; /* 16MHz crystal on WaveShare HAT */
                clock-accuracy = <100>;
            };
        };
    };
    
    /* Configure SPI1 for first MCP2515 */
    fragment@1 {
        target = <&spi1>;
        __overlay__ {
            status = "okay";
            #address-cells = <1>;
            #size-cells = <0>;
            
            mcp2515_can0: mcp2515@0 {
                compatible = "microchip,mcp2515";
                reg = <0>;
                spi-max-frequency = <10000000>;
                interrupt-parent = <&tegra_main_gpio>;
                interrupts = <TEGRA234_MAIN_GPIO(Q, 5) IRQ_TYPE_LEVEL_LOW>; /* PIN 29 */
                clocks = <&can_clock>;
                nvidia,enable-hw-based-cs;
                controller-data {
                    nvidia,cs-setup-clk-count = <0x1e>;
                    nvidia,cs-hold-clk-count = <0x1e>;
                    nvidia,rx-clk-tap-delay = <0x10>;
                    nvidia,tx-clk-tap-delay = <0x0>;
                };
            };
        };
    };
    
    /* Configure SPI3 for second MCP2515 */
    fragment@2 {
        target = <&spi3>;
        __overlay__ {
            status = "okay";
            #address-cells = <1>;
            #size-cells = <0>;
            
            mcp2515_can1: mcp2515@0 {
                compatible = "microchip,mcp2515";
                reg = <0>;
                spi-max-frequency = <10000000>;
                interrupt-parent = <&tegra_main_gpio>;
                interrupts = <TEGRA234_MAIN_GPIO(H, 0) IRQ_TYPE_LEVEL_LOW>; /* PIN 33 */
                clocks = <&can_clock>;
                nvidia,enable-hw-based-cs;
                controller-data {
                    nvidia,cs-setup-clk-count = <0x1e>;
                    nvidia,cs-hold-clk-count = <0x1e>;
                    nvidia,rx-clk-tap-delay = <0x10>;
                    nvidia,tx-clk-tap-delay = <0x0>;
                };
            };
        };
    };
};
EOF

echo -e "\n${YELLOW}Now you need to enable SPI and configure pins using Jetson-IO${NC}"
echo "Would you like to run Jetson-IO now? (y/n)"
read -r response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
    /opt/nvidia/jetson-io/jetson-io.py
    echo -e "${YELLOW}Please complete these steps in Jetson-IO:${NC}"
    echo "1. Select 'Configure Jetson Expansion Header pins'"
    echo "2. Enable SPI1 and SPI3 interfaces and configure proper pins"
    echo "3. Save and reboot"
else
    echo -e "${YELLOW}Please run Jetson-IO manually with:${NC}"
    echo "sudo /opt/nvidia/jetson-io/jetson-io.py"
    echo "And enable SPI1 and SPI3 interfaces"
fi

# Create setup script
echo -e "\n${YELLOW}Creating CAN setup script...${NC}"
cat > /usr/local/bin/setup_mcp2515_can.sh << 'EOF'
#!/bin/bash

# Load required modules
modprobe can
modprobe can_raw
modprobe can_dev
modprobe mcp251x

# Wait for device initialization
sleep 2

# Configure CAN interfaces
ip link set can0 down 2>/dev/null
ip link set can1 down 2>/dev/null

# Set bitrate to 100kbps (standard for testing)
ip link set can0 type can bitrate 100000
ip link set can1 type can bitrate 100000

# Bring up interfaces
ip link set can0 up
ip link set can1 up

# Set TX queue length
ifconfig can0 txqueuelen 65536
ifconfig can1 txqueuelen 65536

echo "CAN interfaces configured successfully!"
EOF
chmod +x /usr/local/bin/setup_mcp2515_can.sh

# Create systemd service
echo -e "\n${YELLOW}Creating systemd service...${NC}"
cat > /etc/systemd/system/can-hat-setup.service << EOF
[Unit]
Description=Setup CAN interfaces for WaveShare 2-CH CAN HAT
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup_mcp2515_can.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
systemctl enable can-hat-setup.service

echo -e "\n${GREEN}WaveShare 2-CH CAN HAT setup complete!${NC}"
echo "Next steps:"
echo "1. Configure Jetson-IO to enable SPI interfaces if you haven't already"
echo "2. Reboot your system"
echo "3. After reboot, run the test script: sudo /home/cavalla/Unicarrier_MXST18C-2/src/can_bus/test_can_hat.sh"
echo "4. Remember to connect CAN0_H to CAN1_H and CAN0_L to CAN1_L for testing"

exit 0 