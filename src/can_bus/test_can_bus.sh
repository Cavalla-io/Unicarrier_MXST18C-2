#!/bin/bash

# Source ROS2 workspace
if [ -f "/home/cavalla/Unicarrier_MXST18C-2/install/setup.bash" ]; then
    source /home/cavalla/Unicarrier_MXST18C-2/install/setup.bash
else
    echo "Error: Workspace setup.bash not found. Make sure to build the workspace first."
    exit 1
fi

# Run the test script with all arguments passed to this script
python3 /home/cavalla/Unicarrier_MXST18C-2/src/can_bus/tests/test_can_bus.py "$@" 