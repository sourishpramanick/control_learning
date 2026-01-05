# Control Learning ROS2 Package

This ROS2 package enables simulation of the differential drive robot defined in `model.cpp` within the ROS2 ecosystem.

## Overview

The `control_learning_ros2` package provides:
- A ROS2 node (`robot_simulator_node`) that simulates the robot dynamics using the CasADi-based model
- URDF description of the differential drive robot
- Launch files for easy startup
- RViz2 configuration for visualization

## Robot Model

The robot is a differential drive robot with:
- **States**: `x`, `y`, `theta` (position and orientation)
- **Controls**: `v` (linear velocity), `omega` (angular velocity)
- **Update rate**: 20Hz (50ms discretization step)

## Dependencies

### ROS2 Dependencies
- `rclcpp` - ROS2 C++ client library
- `std_msgs` - Standard ROS2 messages
- `geometry_msgs` - Geometry messages (Twist, TransformStamped)
- `nav_msgs` - Navigation messages (Odometry)
- `tf2` and `tf2_ros` - Transform library
- `robot_state_publisher` - Publishes robot state from URDF
- `rviz2` - Visualization tool

### External Dependencies
- `casadi` - For robot dynamics computation
- `fmt` - Formatting library

## Installation

### 1. Install ROS2 Jazzy (Ubuntu 24.04)

Follow the official ROS2 installation guide:
```bash
# Set up sources
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 2. Install CasADi

```bash
# Install CasADi dependencies
sudo apt install -y coinor-libipopt-dev

# Download and install CasADi (or install via your preferred method)
# See: https://web.casadi.org/get/
```

### 3. Source ROS2 Environment

```bash
source /opt/ros/jazzy/setup.bash
```

### 4. Build the Package

```bash
# Navigate to the ROS2 workspace
cd ros2_ws

# Build the package
colcon build --packages-select control_learning_ros2

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the Simulation

To start the robot simulator with visualization:

```bash
ros2 launch control_learning_ros2 robot_sim.launch.py
```

To start without RViz2:

```bash
ros2 launch control_learning_ros2 robot_sim.launch.py use_rviz:=false
```

### Control the Robot

Send velocity commands to the robot using the `/cmd_vel` topic:

```bash
# Move forward at 1 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate at 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Move forward and rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

### Monitor Robot State

View odometry data:

```bash
ros2 topic echo /odom
```

View TF transforms:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

### Visualize in RViz2

If you launched without RViz2 initially, you can start it separately:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix control_learning_ros2)/share/control_learning_ros2/config/robot_sim.rviz
```

## Topics

### Published Topics

- `/odom` (`nav_msgs/msg/Odometry`) - Robot odometry at 20Hz
- `/tf` - Transform from `odom` to `base_link`
- `/robot_description` - Robot URDF description

### Subscribed Topics

- `/cmd_vel` (`geometry_msgs/msg/Twist`) - Velocity commands

## Architecture

```
┌─────────────────────┐
│   User/Controller   │
│                     │
└──────────┬──────────┘
           │ /cmd_vel
           │ (Twist)
           ▼
┌─────────────────────────────────┐
│  Robot Simulator Node           │
│  - Uses model.cpp for dynamics  │
│  - Integrates state at 20Hz     │
│  - Publishes odometry & TF      │
└──────────┬──────────────────────┘
           │ /odom (Odometry)
           │ /tf (odom→base_link)
           ▼
┌─────────────────────────────────┐
│       RViz2 Visualization       │
│  - Shows robot model            │
│  - Shows odometry trail         │
│  - Shows TF frames              │
└─────────────────────────────────┘
```

## Development

### File Structure

```
ros2_ws/src/control_learning_ros2/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package dependencies
├── README.md                # This file
├── src/
│   └── robot_simulator_node.cpp  # Main simulation node
├── launch/
│   └── robot_sim.launch.py      # Launch file
├── urdf/
│   └── robot.urdf               # Robot description
└── config/
    └── robot_sim.rviz           # RViz configuration
```

### Extending the Simulator

The robot dynamics are defined in `../../../src/cpp/robot/dynamics/model.cpp`. To modify the robot model:

1. Update the dynamics equations in `model.cpp`
2. Rebuild the package: `colcon build --packages-select control_learning_ros2`
3. Re-launch the simulation

## Troubleshooting

### CasADi not found

If CMake cannot find CasADi:
```bash
# Make sure CasADi is installed system-wide or set CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH=/path/to/casadi:$CMAKE_PREFIX_PATH
```

### Robot not moving

1. Check if velocity commands are being published:
   ```bash
   ros2 topic hz /cmd_vel
   ```

2. Check if the simulator node is running:
   ```bash
   ros2 node list
   ```

3. Check node logs:
   ```bash
   ros2 run control_learning_ros2 robot_simulator_node --ros-args --log-level debug
   ```

### TF errors in RViz2

Make sure the robot_state_publisher node is running:
```bash
ros2 node list | grep robot_state_publisher
```

## License

MIT

## Contributing

Contributions are welcome! Please ensure your changes:
1. Follow the existing code style
2. Include appropriate documentation
3. Pass all build tests
