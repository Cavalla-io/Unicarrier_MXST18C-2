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

## License

[Specify license information here]

## Contributors

[List contributors here] 