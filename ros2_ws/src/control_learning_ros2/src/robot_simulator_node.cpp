/**
 * @file robot_simulator_node.cpp
 * @brief ROS2 node for simulating the differential drive robot
 * 
 * This node subscribes to velocity commands (/cmd_vel) and publishes:
 * - Odometry messages (/odom)
 * - TF transforms (odom -> base_link)
 * 
 * The robot dynamics are computed using the Model class from model.cpp
 */

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "model.hpp"

using namespace std::chrono_literals;

class RobotSimulatorNode : public rclcpp::Node
{
public:
    RobotSimulatorNode()
        : Node("robot_simulator_node"),
          robot_model_(std::vector<double>{1.0, 2.0, 3.0}),
          x_(0.0), y_(0.0), theta_(0.0),
          v_(0.0), omega_(0.0),
          disc_step_size_(0.05)  // 50ms update rate -> 20Hz
    {
        // Create publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // Create subscriber for velocity commands
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&RobotSimulatorNode::cmdVelCallback, this, std::placeholders::_1));
        
        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Create timer for simulation update (20Hz)
        timer_ = this->create_wall_timer(
            50ms, std::bind(&RobotSimulatorNode::simulationUpdate, this));
        
        RCLCPP_INFO(this->get_logger(), "Robot Simulator Node initialized");
        RCLCPP_INFO(this->get_logger(), "Initial state: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract velocity commands
        v_ = msg->linear.x;
        omega_ = msg->angular.z;
        
        RCLCPP_DEBUG(this->get_logger(), "Received cmd_vel: v=%.2f, omega=%.2f", v_, omega_);
    }

    void simulationUpdate()
    {
        // Prepare current state
        std::vector<double> current_state = {x_, y_, theta_};
        std::vector<double> controls = {v_, omega_};
        
        // Compute next state using the robot model
        auto next_state = robot_model_.getDiscretizedDynamics()(casadi::DMVector{
            casadi::DM(current_state),
            casadi::DM(controls),
            casadi::DM(disc_step_size_)
        })[0];
        
        // Update state
        x_ = static_cast<double>(next_state(0));
        y_ = static_cast<double>(next_state(1));
        theta_ = static_cast<double>(next_state(2));
        
        // Publish odometry
        publishOdometry();
        
        // Publish TF transform
        publishTransform();
    }

    void publishOdometry()
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // Set position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Set orientation (convert theta to quaternion)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // Set velocities
        odom_msg.twist.twist.linear.x = v_;
        odom_msg.twist.twist.angular.z = omega_;
        
        odom_pub_->publish(odom_msg);
    }

    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
    }

    // Robot model
    robot::Model robot_model_;
    
    // State variables
    double x_, y_, theta_;
    
    // Control inputs
    double v_, omega_;
    
    // Discretization step size
    double disc_step_size_;
    
    // ROS2 components
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
