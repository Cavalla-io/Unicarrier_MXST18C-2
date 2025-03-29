# Forklift Control ROS2 Workspace

This repository contains ROS2 packages for forklift teleoperation and camera processing.

## Packages

### 1. teleop_forklift_control
A package for teleoperating forklift controls including drive and lift functionality.

### 2. camera_pipelines
Contains pipelines for processing camera data. **Note: This package is currently non-functional and is a work in progress.**

## Prerequisites

- Ubuntu 22.04 (or compatible Linux distribution)
- ROS2 Humble (or a compatible ROS2 distribution)
- Python 3.10+

## Building the Workspace

1. Clone the repository:
   ```bash
   git clone https://github.com/Cavalla-io/Unicarrier_MXST18C-2.git ros2_ws
   cd ros2_ws
   ```

2. Install dependencies:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Packages

### Teleop Forklift Control

Launch the forklift teleop control:
```bash
ros2 launch teleop_forklift_control forklift_teleop.launch.py
```

**Note:** The wheel tracker functionality is integrated within the forklift_teleop.launch.py file. There is no separate wheel_tracker launch file.

#### Control Parameters:
- Drive controller controls forklift movement
- Lift controller manages fork height
- Wheel tracker provides wheel position feedback

### Camera Pipelines

**Note: The camera pipeline is currently under development and not yet functional.**

Run the camera nodes (when available):
```bash
ros2 run camera_pipelines [node_name]
```

## CAN Bus Setup

This workspace includes scripts to set up and manage CAN bus interfaces:

### Initial Setup (one-time)

Run the network privileges setup script with sudo:
```bash
sudo ./setup_network_privileges.sh
```

This will:
1. Grant your user the necessary permissions to manage network interfaces
2. Set up a systemd service to automatically configure the CAN bus at startup
3. Install required CAN utilities

You may need to log out and log back in for group changes to take effect.

### Using the CAN Tools

After the setup, you can use the CAN tools script without sudo:
```bash
./can_tools.sh status    # Check CAN interface status
./can_tools.sh start     # Start the CAN interface
./can_tools.sh stop      # Stop the CAN interface
./can_tools.sh restart   # Restart the CAN interface
./can_tools.sh monitor   # Monitor CAN traffic
./can_tools.sh send 123#DEADBEEF  # Send a CAN frame
./can_tools.sh install   # Install CAN utilities if needed
```

CAN Bus Configuration:
- Type: can
- Bitrate: 250000
- TX Queue Length: 10000

## Development

### Workspace Structure
```
ros2_ws/
├── src/
│   ├── camera_pipelines/
│   ├── teleop_forklift_control/
├── build/          # Build artifacts (ignored by git)
├── install/        # Install space (ignored by git)
├── log/            # Log files (ignored by git)
├── can_tools.sh    # Script for CAN bus operations
├── setup_network_privileges.sh  # Setup script for network permissions
```

### Adding New Packages

To create a new package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_package
```

## Troubleshooting

If you encounter issues:

1. Make sure all dependencies are installed:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Ensure the workspace is properly sourced:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

3. Check ROS2 environment:
   ```bash
   printenv | grep -i ROS
   ```

4. To see available launch files for a package:
   ```bash
   ros2 pkg prefix teleop_forklift_control
   ls $(ros2 pkg prefix teleop_forklift_control)/share/teleop_forklift_control/launch
   ```

5. If having CAN bus issues:
   ```bash
   ./can_tools.sh status    # Check the status of CAN interface
   sudo systemctl status can-setup.service  # Check if the service is running
   ```

## Contributors

[List contributors here] 
