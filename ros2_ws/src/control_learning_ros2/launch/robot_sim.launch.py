"""
Launch file for the differential drive robot simulator

This launch file starts:
1. Robot state publisher (for URDF visualization)
2. Robot simulator node (for dynamics simulation)
3. RViz2 (for visualization) - optional
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('control_learning_ros2')
    
    # Get URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Declare launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2')
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # Robot simulator node
    robot_simulator_node = Node(
        package='control_learning_ros2',
        executable='robot_simulator_node',
        name='robot_simulator_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # RViz2 node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_share, 'config', 'robot_sim.rviz')]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_rviz_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(robot_simulator_node)
    ld.add_action(rviz_node)
    
    return ld
