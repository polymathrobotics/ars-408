#!/bin/bash

# Source ROS2 workspace
source "/opt/polymathrobotics/setup.bash"

# Start Xvfb
Xvfb :99 -screen 0 1024x768x24 &

# Execute the passed command
exec "$@"
