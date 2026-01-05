/**
 * @file simple_controller.cpp
 * @brief Example ROS2 node that publishes velocity commands
 * 
 * This is a simple example showing how to create a ROS2 node that
 * controls the robot by publishing to /cmd_vel topic.
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/**
 * @class SimpleController
 * @brief A simple controller that makes the robot move in a circular pattern
 */
class SimpleController : public rclcpp::Node
{
public:
    SimpleController()
        : Node("simple_controller")
    {
        // Create publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Create timer to publish commands at 10Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SimpleController::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Simple Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Robot will move in a circular pattern");
    }

private:
    void timerCallback()
    {
        auto msg = geometry_msgs::msg::Twist();
        
        // Move forward at 0.5 m/s and rotate at 0.3 rad/s
        // This creates a circular motion
        msg.linear.x = 0.5;
        msg.angular.z = 0.3;
        
        cmd_vel_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleController>();
    
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to stop");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
