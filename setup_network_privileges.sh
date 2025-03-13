#!/bin/bash

# setup_network_privileges.sh - Set up network privileges for a non-root user
# This allows a regular user to manage network interfaces, access all ports, and modify network configs
# Must be run with root privileges initially

if [ "$(id -u)" -ne 0 ]; then
    echo "This script requires root privileges. Please run with sudo."
    exit 1
fi

# Get the username to grant privileges to (default to current sudo user)
if [ -z "$1" ]; then
    USERNAME=$(logname || echo $SUDO_USER || echo $USER)
else
    USERNAME="$1"
fi

echo "Setting up network privileges for user: $USERNAME"

# Check if user exists
if ! id "$USERNAME" &>/dev/null; then
    echo "Error: User $USERNAME does not exist."
    exit 1
fi

# 1. Add user to relevant groups
echo "Adding $USERNAME to network-related groups..."
for GROUP in netdev dialout plugdev; do
    if getent group $GROUP > /dev/null; then
        usermod -a -G $GROUP $USERNAME
        echo "  - Added to $GROUP group"
    else
        echo "  - Group $GROUP not found, skipping"
    fi
done

# 2. Set up capabilities for network access
echo "Setting up network capabilities..."

# Create capability script for binding to privileged ports
CAP_SCRIPT="/usr/local/bin/setup_net_capabilities.sh"
cat > "$CAP_SCRIPT" << 'EOF'
#!/bin/bash
# Set capabilities for network utilities
for BINARY in /bin/ip /sbin/ip /usr/sbin/ip /bin/ifconfig /sbin/ifconfig /usr/sbin/ifconfig; do
    if [ -f "$BINARY" ]; then
        setcap cap_net_admin+ep "$BINARY"
    fi
done

# Allow binding to privileged ports
for CMD in /usr/bin/node /usr/bin/python3 /usr/bin/python; do
    if [ -f "$CMD" ]; then
        setcap 'cap_net_bind_service=+ep' "$CMD"
    fi
done
EOF

chmod +x "$CAP_SCRIPT"
$CAP_SCRIPT

# 3. Create sudoers entry for network commands without password
echo "Setting up sudo rules for network commands..."
SUDOERS_FILE="/etc/sudoers.d/network_privileges"

cat > "$SUDOERS_FILE" << EOF
# Allow $USERNAME to run network commands without password
$USERNAME ALL=(ALL) NOPASSWD: /sbin/ip, /bin/ip, /usr/sbin/ip
$USERNAME ALL=(ALL) NOPASSWD: /sbin/ifconfig, /bin/ifconfig, /usr/sbin/ifconfig
$USERNAME ALL=(ALL) NOPASSWD: /usr/bin/netstat, /bin/netstat
$USERNAME ALL=(ALL) NOPASSWD: /sbin/route, /bin/route
$USERNAME ALL=(ALL) NOPASSWD: /usr/sbin/iptables, /sbin/iptables
$USERNAME ALL=(ALL) NOPASSWD: /usr/bin/tee /etc/network/*, /usr/bin/tee /etc/netplan/*
$USERNAME ALL=(ALL) NOPASSWD: /usr/bin/networkctl, /bin/networkctl
$USERNAME ALL=(ALL) NOPASSWD: /bin/systemctl restart NetworkManager.service
$USERNAME ALL=(ALL) NOPASSWD: /bin/systemctl restart networking.service
$USERNAME ALL=(ALL) NOPASSWD: /bin/systemctl restart systemd-networkd.service
$USERNAME ALL=(ALL) NOPASSWD: /usr/bin/apt-get update, /usr/bin/apt-get install *
$USERNAME ALL=(ALL) NOPASSWD: $CAP_SCRIPT
EOF

chmod 0440 "$SUDOERS_FILE"

# 4. Set up udev rules for CAN and network devices
echo "Setting up udev rules for network device access..."
UDEV_RULES_FILE="/etc/udev/rules.d/99-network-privileges.rules"

cat > "$UDEV_RULES_FILE" << EOF
# Grant permissions for CAN devices
SUBSYSTEM=="net", ACTION=="add", KERNEL=="can*", TAG+="systemd", ENV{SYSTEMD_WANTS}="can-setup.service"
KERNEL=="can*", SUBSYSTEM=="net", ACTION=="add", RUN+="/bin/sh -c 'setfacl -m u:$USERNAME:rw /dev/\$name'"

# Grant permissions for network devices
SUBSYSTEM=="net", ACTION=="add", RUN+="/bin/sh -c 'chmod 666 /sys/class/net/\$name/flags || true'"
EOF

# Reload udev rules
echo "Reloading udev rules..."
udevadm control --reload-rules

# 5. Install CAN-setup service (combines install_can_service.sh functionality)
if [ -d "/etc/systemd/system" ]; then
    echo "Setting up CAN bus systemd service..."
    
    # Create the CAN setup service
    CAN_SETUP_SERVICE="/etc/systemd/system/can-setup.service"
    
    cat > "$CAN_SETUP_SERVICE" << EOF
[Unit]
Description=CAN Bus Setup
After=network.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'modprobe can && modprobe can_raw && modprobe can_dev && \
                        ip link set can0 down || true && \
                        ip link add dev can0 type can || true && \
                        ip link set can0 type can bitrate 250000 && \
                        ip link set can0 txqueuelen 10000 && \
                        ip link set can0 up'
RemainAfterExit=yes
ExecStop=/bin/bash -c 'ip link set can0 down'

[Install]
WantedBy=multi-user.target
EOF

    # Create the CAN permissions service
    CAN_PERMISSIONS_SERVICE="/etc/systemd/system/can-permissions.service"
    
    cat > "$CAN_PERMISSIONS_SERVICE" << EOF
[Unit]
Description=Set permissions for CAN interfaces
After=can-setup.service
Requires=can-setup.service

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'chown root:$USERNAME /sys/class/net/can* 2>/dev/null || true'
ExecStart=/bin/sh -c 'chmod g+rw /sys/class/net/can* 2>/dev/null || true'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

    # Enable and start both services
    systemctl daemon-reload
    systemctl enable can-setup.service
    systemctl enable can-permissions.service
    
    # Start the services if they're not already running
    systemctl start can-setup.service
    systemctl start can-permissions.service
    
    echo "CAN services installed and started."
fi

# 6. Install can-utils if not already installed
if ! command -v candump &>/dev/null || ! command -v cansend &>/dev/null; then
    echo "Installing can-utils package..."
    apt-get update && apt-get install -y can-utils
    echo "can-utils installed successfully."
fi

echo ""
echo "Network privileges setup complete for user $USERNAME."
echo "The user can now manage network interfaces and access all ports without using sudo."
echo "You may need to log out and log back in for group changes to take effect."
echo ""
echo "To manage CAN bus: ./can_tools.sh"

exit 0 