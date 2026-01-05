#!/bin/bash

# Exit on any error
set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== ROS2 Package Build Script ===${NC}"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS2 environment not sourced!${NC}"
    echo "Please source your ROS2 installation first:"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

echo -e "${GREEN}Found ROS2 distribution: $ROS_DISTRO${NC}"

# Navigate to workspace
cd "$(dirname "$0")/ros2_ws"

echo -e "${BLUE}=== Cleaning previous build ===${NC}"
rm -rf build install log

echo -e "${BLUE}=== Building ROS2 package ===${NC}"
colcon build --packages-select control_learning_ros2 --symlink-install

if [ $? -eq 0 ]; then
    echo -e "${GREEN}=== Build successful! ===${NC}"
    echo ""
    echo "To use the package, source the workspace:"
    echo "  source ros2_ws/install/setup.bash"
    echo ""
    echo "Then launch the simulation:"
    echo "  ros2 launch control_learning_ros2 robot_sim.launch.py"
else
    echo -e "${RED}=== Build failed! ===${NC}"
    exit 1
fi
