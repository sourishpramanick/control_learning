# Control Learning

A control learning project featuring a differential drive robot with CasADi-based dynamics modeling.

## Overview

This repository contains:
- **C++ Implementation**: Robot dynamics model using CasADi for symbolic computation
- **Python Implementation**: Python bindings and utilities
- **ROS2 Support**: Full ROS2 integration for robot simulation and visualization

## Robot Model

The differential drive robot has:
- **States**: 
  - `x`: Position in x-axis (m)
  - `y`: Position in y-axis (m)
  - `theta`: Orientation (rad)
- **Controls**:
  - `v`: Linear velocity (m/s)
  - `omega`: Angular velocity (rad/s)

The dynamics are computed using CasADi and discretized using Runge-Kutta integration.

## Project Structure

```
.
├── src/
│   ├── cpp/                    # C++ implementation
│   │   ├── main.cpp           # Standalone demo
│   │   └── robot/
│   │       └── dynamics/      # Robot dynamics model
│   │           ├── model.cpp
│   │           └── include/
│   │               └── model.hpp
│   └── python/                # Python implementation
├── ros2_ws/                   # ROS2 workspace
│   └── src/
│       └── control_learning_ros2/  # ROS2 package
│           ├── src/           # ROS2 nodes
│           ├── launch/        # Launch files
│           ├── urdf/          # Robot description
│           └── config/        # Configuration files
├── CMakeLists.txt            # Main CMake configuration
└── conanfile.txt             # Conan dependencies
```

## Getting Started

### Option 1: Standalone C++ Build

#### Prerequisites
- CMake 3.20+
- C++17 compiler
- Conan package manager
- CasADi library

#### Build and Run
```bash
./build_and_run.sh
```

Or manually:
```bash
# Install dependencies
conan install . --output-folder=. --build=missing

# Configure
cmake --preset wsl-conan-release

# Build
cmake --build build/Release

# Run
./build/Release/control_learning
```

### Option 2: ROS2 Integration

For ROS2 simulation and visualization, see the [ROS2 package README](ros2_ws/src/control_learning_ros2/README.md).

Quick start:
```bash
# Install ROS2 Jazzy (Ubuntu 24.04)
# See ROS2 installation guide

# Build the ROS2 package
cd ros2_ws
colcon build --packages-select control_learning_ros2
source install/setup.bash

# Launch the simulation
ros2 launch control_learning_ros2 robot_sim.launch.py

# In another terminal, control the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

## Features

### Standalone Mode
- Direct computation of robot dynamics
- Discretized dynamics using Runge-Kutta method
- CasADi symbolic expressions for efficient computation

### ROS2 Mode
- Real-time robot simulation at 20Hz
- Odometry publishing on `/odom` topic
- TF transform broadcasting (`odom` → `base_link`)
- Velocity command subscription on `/cmd_vel`
- URDF-based robot visualization
- RViz2 integration with pre-configured views

## Dependencies

### Core Dependencies
- **CasADi**: Symbolic framework for algorithmic differentiation
- **fmt**: Modern formatting library

### ROS2 Dependencies (for ROS2 mode only)
- ROS2 Jazzy (Ubuntu 24.04)
- rclcpp
- geometry_msgs
- nav_msgs
- tf2
- robot_state_publisher
- rviz2

## Examples

### Standalone C++ Example

```cpp
#include "robot/dynamics/include/model.hpp"

int main() {
    // Create robot model
    std::vector<double> params = {1.0, 2.0, 3.0};
    robot::Model bot{std::move(params)};
    
    // Define initial state and controls
    std::vector<double> initState = {0.0, 0.0, 0.0};  // x, y, theta
    std::vector<double> controls = {1.0, 0.1};         // v, omega
    
    // Simulate one step
    double disc_step_size = 0.1;
    auto next_state = bot.getDiscretizedDynamics()(casadi::DMVector{
        casadi::DM(initState),
        casadi::DM(controls),
        casadi::DM(disc_step_size)
    })[0];
    
    std::cout << "Next state: " << next_state << std::endl;
    return 0;
}
```

### ROS2 Example

```bash
# Terminal 1: Launch simulation
ros2 launch control_learning_ros2 robot_sim.launch.py

# Terminal 2: Move the robot in a circle
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.5}}" -r 10

# Terminal 3: Monitor odometry
ros2 topic echo /odom

# Terminal 4: Visualize TF tree
ros2 run tf2_tools view_frames
```

## Development

### Building Only the Robot Dynamics Library

```bash
cd build/Release
cmake --build . --target dynamics
```

### Running Tests

(Tests to be added)

## Contributing

Contributions are welcome! Please ensure:
1. Code follows the existing style
2. All builds pass successfully
3. Documentation is updated as needed

## License

MIT

## Acknowledgments

- Uses CasADi for symbolic computation and automatic differentiation
- ROS2 integration follows standard ROS2 patterns and conventions
