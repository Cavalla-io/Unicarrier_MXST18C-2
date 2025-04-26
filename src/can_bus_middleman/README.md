# CAN Bus Middleman

A basic repository for CAN bus communication and hacking using the Waveshare 2CH CAN HAT with Raspberry Pi or Jetson devices.

## Hardware Requirements

- [Waveshare 2-CH CAN HAT](https://www.waveshare.com/2-ch-can-hat.htm)
- Raspberry Pi (3/4/5) or Jetson device
- MicroSD card with Raspberry Pi OS/Linux
- Power supply for your Raspberry Pi/Jetson
- CAN bus devices to interact with

## Hardware Setup

1. Attach the Waveshare 2-CH CAN HAT to your Raspberry Pi or Jetson device's GPIO pins
2. Connect your CAN devices to the CAN terminals on the HAT
   - The HAT provides two independent CAN channels (CAN0 and CAN1)
   - Each channel has CANH, CANL and GND connections
3. Power on your Raspberry Pi/Jetson device

## Software Setup

### Enable SPI Interface

For Raspberry Pi:
```bash
sudo raspi-config
```
Navigate to "Interface Options" > "SPI" > "Yes" to enable SPI

### Install Required Packages

```bash
sudo apt-get update
sudo apt-get install -y can-utils python3-pip
pip3 install python-can
```

### Setup CAN HAT Driver

1. Download the necessary utilities:
```bash
git clone https://github.com/waveshare/2-CH-CAN-HAT
cd 2-CH-CAN-HAT
```

2. Install the driver:
```bash
sudo ./install.sh
```

3. Reboot your device:
```bash
sudo reboot
```

### Using this Repository

1. Clone this repository:
```bash
git clone https://github.com/yourusername/can_bus_middleman.git
cd can_bus_middleman
```

2. Initialize the CAN interfaces using the provided script:
```bash
python3 can_bus_setup.py
```

This script includes functions to:
- Check CAN interface states
- Reset CAN interfaces
- Set optimal buffer sizes
- Handle CAN startup and shutdown

## Basic Usage

### Start the CAN interfaces

```python
import can_bus_setup

# Start CAN interfaces
can_bus_setup.can_startup()
```

### Send and Receive CAN Messages

```python
import can

# Create a CAN bus interface
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Send a message
msg = can.Message(arbitration_id=0x123, data=[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08], is_extended_id=False)
bus.send(msg)

# Receive messages
for msg in bus:
    print(f"Received message: ID={msg.arbitration_id:X}, Data={msg.data}")
```

### Shutdown CAN interfaces

```python
import can_bus_setup

# Shutdown CAN interfaces
can_bus_setup.can_shutdown()
```

## Using can-utils (Command Line)

The `can-utils` package provides useful command-line tools:

```bash
# Display CAN traffic
candump can0

# Send a CAN frame
cansend can0 123#01020304

# Generate CAN traffic
cangen can0

# CAN bus sniffer with more details
candump -cae can0,0:0,#FFFFFFFF
```

## Troubleshooting

If you encounter issues with the CAN interfaces:

1. Check if interfaces are up:
```bash
ip -details link show can0
ip -details link show can1
```

2. Reset an interface manually:
```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 250000
```

3. Check system logs:
```bash
dmesg | grep -i can
```

## Resources

- [Waveshare 2-CH CAN HAT Wiki](https://www.waveshare.com/wiki/2-CH_CAN_HAT)
- [python-can Documentation](https://python-can.readthedocs.io/en/stable/)
- [SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html) 