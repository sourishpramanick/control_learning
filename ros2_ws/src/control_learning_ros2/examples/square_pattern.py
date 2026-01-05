#!/usr/bin/env python3
"""
Simple example script to control the robot in a square pattern.

This script demonstrates:
1. Publishing velocity commands to /cmd_vel
2. Creating timed movement sequences
3. Basic robot control patterns
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


class SquarePatternController(Node):
    """
    A simple controller that makes the robot move in a square pattern.
    """
    
    def __init__(self):
        super().__init__('square_pattern_controller')
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Square Pattern Controller initialized')
        
    def move_forward(self, duration, speed=0.5):
        """Move forward at given speed for specified duration."""
        self.get_logger().info(f'Moving forward at {speed} m/s for {duration}s')
        
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            rate.sleep()
    
    def rotate(self, duration, angular_speed=0.5):
        """Rotate at given angular speed for specified duration."""
        self.get_logger().info(f'Rotating at {angular_speed} rad/s for {duration}s')
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            rate.sleep()
    
    def stop(self):
        """Stop the robot."""
        self.get_logger().info('Stopping robot')
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(msg)
    
    def move_square(self, side_length=2.0, linear_speed=0.5):
        """
        Move the robot in a square pattern.
        
        Args:
            side_length: Length of each side in meters
            linear_speed: Speed while moving forward in m/s
        """
        self.get_logger().info(f'Starting square pattern with side length {side_length}m')
        
        # Calculate movement duration for each side
        forward_duration = side_length / linear_speed
        
        # Calculate rotation duration (90 degrees = π/2 radians)
        angular_speed = 0.5  # rad/s
        rotation_duration = (math.pi / 2.0) / angular_speed
        
        # Move in a square (4 sides)
        for i in range(4):
            self.get_logger().info(f'Square side {i+1}/4')
            
            # Move forward
            self.move_forward(forward_duration, linear_speed)
            
            # Short pause
            time.sleep(0.5)
            
            # Rotate 90 degrees
            self.rotate(rotation_duration, angular_speed)
            
            # Short pause
            time.sleep(0.5)
        
        # Stop at the end
        self.stop()
        
        self.get_logger().info('Square pattern complete!')


def main(args=None):
    rclpy.init(args=args)
    
    controller = SquarePatternController()
    
    try:
        # Wait a bit for everything to initialize
        time.sleep(2.0)
        
        # Execute square pattern
        controller.move_square(side_length=1.0, linear_speed=0.3)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
