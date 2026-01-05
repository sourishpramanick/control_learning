# ROS2 Control Learning Examples

This directory contains example programs demonstrating how to control the differential drive robot.

## Available Examples

### 1. Simple Controller (C++)

**File**: `simple_controller.cpp`

A basic C++ example that makes the robot move in a circular pattern.

**Usage**:
```bash
# After building the package
ros2 run control_learning_ros2 simple_controller
```

**What it does**:
- Publishes velocity commands to `/cmd_vel` at 10Hz
- Sets linear velocity to 0.5 m/s
- Sets angular velocity to 0.3 rad/s
- Creates a circular motion pattern

### 2. Square Pattern (Python)

**File**: `square_pattern.py`

A Python example that makes the robot move in a square pattern.

**Usage**:
```bash
# After building the package
ros2 run control_learning_ros2 square_pattern.py
```

**What it does**:
- Moves the robot in a square pattern
- Each side is 1 meter long
- Uses timed movements and rotations
- Demonstrates sequential control logic

## Running Examples

### Prerequisites

1. Start the robot simulator:
```bash
ros2 launch control_learning_ros2 robot_sim.launch.py
```

2. In another terminal, run an example:
```bash
# C++ example
ros2 run control_learning_ros2 simple_controller

# OR Python example
ros2 run control_learning_ros2 square_pattern.py
```

### Stopping Examples

Press `Ctrl+C` to stop any running example.

## Creating Your Own Controller

### Python Controller Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward velocity
        msg.angular.z = 0.2  # Angular velocity
        self.cmd_vel_pub.publish(msg)

def main():
    rclpy.init()
    controller = MyController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Controller Template

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MyController : public rclcpp::Node
{
public:
    MyController() : Node("my_controller")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&MyController::timerCallback, this));
    }

private:
    void timerCallback()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.5;  // Forward velocity
        msg.angular.z = 0.2;  // Angular velocity
        cmd_vel_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyController>());
    rclcpp::shutdown();
    return 0;
}
```

## Advanced Examples Ideas

Here are some ideas for more advanced controllers:

1. **PID Controller**: Follow a reference trajectory using PID control
2. **Waypoint Navigator**: Navigate through a series of waypoints
3. **Obstacle Avoidance**: Add laser scanner and implement reactive control
4. **Path Tracker**: Follow a pre-defined path using pure pursuit or similar
5. **Teleoperation**: Create a custom teleop interface with speed limits
6. **State Machine**: Implement different behaviors based on robot state
7. **Formation Control**: Control multiple robots in formation
8. **Model Predictive Control**: Use CasADi for online optimization

## Monitoring During Execution

### View Robot Position
```bash
ros2 topic echo /odom --field pose.pose.position
```

### View Robot Velocity
```bash
ros2 topic echo /odom --field twist.twist
```

### View Command Velocity
```bash
ros2 topic echo /cmd_vel
```

### Plot Data (requires plotjuggler)
```bash
ros2 run plotjuggler plotjuggler
```

## Debugging Tips

1. **Check if simulator is running**:
   ```bash
   ros2 node list
   ```
   Should show `robot_simulator_node`

2. **Check topic connections**:
   ```bash
   ros2 topic info /cmd_vel
   ```

3. **Monitor message rate**:
   ```bash
   ros2 topic hz /cmd_vel
   ros2 topic hz /odom
   ```

4. **View node graph**:
   ```bash
   rqt_graph
   ```

## Contributing

Feel free to add your own examples! Follow these guidelines:
- Keep examples simple and focused on one concept
- Include comments explaining the code
- Update this README with your example
- Test with the simulator before submitting
