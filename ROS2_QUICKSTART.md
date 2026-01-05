# ROS2 Quick Start Guide

This guide will help you quickly get started with simulating the differential drive robot in ROS2.

## Prerequisites

- Ubuntu 24.04 (Noble Numbat)
- ROS2 Jazzy installed
- CasADi library installed

## Installation Steps

### Step 1: Install ROS2 Jazzy

```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Source ROS2
source /opt/ros/jazzy/setup.bash
```

### Step 2: Install Dependencies

```bash
# Install CasADi (adjust based on your system)
sudo apt install -y coinor-libipopt-dev

# Install fmt library
sudo apt install -y libfmt-dev

# For CasADi, you may need to build from source or use a package manager
# See: https://web.casadi.org/get/
```

### Step 3: Build the ROS2 Package

```bash
# Clone the repository (if not already done)
# cd /path/to/control_learning

# Build using the provided script
./build_ros2.sh

# Or build manually
cd ros2_ws
colcon build --packages-select control_learning_ros2
source install/setup.bash
```

## Running the Simulation

### Launch Everything

```bash
# Make sure ROS2 is sourced
source /opt/ros/jazzy/setup.bash

# Source the workspace
cd /path/to/control_learning/ros2_ws
source install/setup.bash

# Launch the simulation with RViz2
ros2 launch control_learning_ros2 robot_sim.launch.py
```

### Launch Without RViz2

```bash
ros2 launch control_learning_ros2 robot_sim.launch.py use_rviz:=false
```

## Controlling the Robot

### Using Command Line

Open a new terminal and send velocity commands:

```bash
# Source ROS2 first
source /opt/ros/jazzy/setup.bash

# Move forward at 1 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" -r 10

# Rotate at 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" -r 10

# Move in a circle (forward + rotation)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.5}}" -r 10

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 10
```

### Using teleop_twist_keyboard

You can also install and use the keyboard teleop tool:

```bash
# Install teleop package
sudo apt install -y ros-jazzy-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Then use the keyboard to control the robot:
- `i` - Move forward
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `q/z` - Increase/decrease max speeds

## Monitoring the Robot

### View Odometry

```bash
# Echo odometry messages
ros2 topic echo /odom

# Check odometry message rate
ros2 topic hz /odom
```

### View TF Transforms

```bash
# Check transform between odom and base_link
ros2 run tf2_ros tf2_echo odom base_link

# Visualize TF tree
ros2 run tf2_tools view_frames
# This creates frames.pdf in the current directory
```

### List All Topics

```bash
ros2 topic list
```

Expected topics:
- `/cmd_vel` - Velocity commands (subscribed)
- `/odom` - Odometry data (published)
- `/robot_description` - Robot URDF (published)
- `/tf` - Transform data (published)

## Visualization in RViz2

When RViz2 is running, you should see:
1. **Grid** - Reference grid in the odom frame
2. **TF Frames** - Shows odom and base_link frames
3. **Robot Model** - 3D visualization of the robot from URDF
4. **Odometry** - Arrow showing robot pose and trail of past positions

## Example Workflow

Here's a complete example workflow:

```bash
# Terminal 1: Launch simulation
source /opt/ros/jazzy/setup.bash
cd /path/to/control_learning/ros2_ws
source install/setup.bash
ros2 launch control_learning_ros2 robot_sim.launch.py

# Terminal 2: Control the robot
source /opt/ros/jazzy/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}" -r 10

# Terminal 3: Monitor odometry
source /opt/ros/jazzy/setup.bash
ros2 topic echo /odom --once

# Terminal 4: Check TF
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros tf2_echo odom base_link
```

## Troubleshooting

### Issue: "Package 'control_learning_ros2' not found"

**Solution**: Make sure you've sourced the workspace:
```bash
cd ros2_ws
source install/setup.bash
```

### Issue: "CasADi not found" during build

**Solution**: Install CasADi or set the CMAKE_PREFIX_PATH:
```bash
export CMAKE_PREFIX_PATH=/path/to/casadi:$CMAKE_PREFIX_PATH
```

### Issue: Robot not visible in RViz2

**Solution**: 
1. Check if robot_state_publisher is running: `ros2 node list`
2. In RViz2, verify that "Fixed Frame" is set to "odom"
3. Check if /robot_description topic is being published: `ros2 topic list`

### Issue: No odometry updates

**Solution**:
1. Check if the simulator node is running: `ros2 node list`
2. Verify /odom topic is publishing: `ros2 topic hz /odom`
3. Send velocity commands to trigger movement

### Issue: Build fails with missing dependencies

**Solution**: Install all ROS2 dependencies:
```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Next Steps

- **Create a controller**: Write a ROS2 node that publishes to /cmd_vel
- **Record data**: Use `ros2 bag` to record odometry data
- **Add sensors**: Extend the URDF and simulator node with sensors
- **Path planning**: Integrate with Nav2 for autonomous navigation

## Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [CasADi Documentation](https://web.casadi.org/)
- [RViz2 User Guide](https://github.com/ros2/rviz)
- [TF2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)
