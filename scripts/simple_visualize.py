#!/usr/bin/env python3
"""
Simple example: Visualize trajectories from JSON
"""

from visualize_trajectories import TrajectoryVisualizer

# Load and visualize with obstacles
viz = TrajectoryVisualizer('ocp_solution.json', 
                          obstacles_file='src/cpp/robot/environment/obstacles.json')

# Show static plot
viz.plot_static(save_path='trajectory_plot.png')

# Show animation (and optionally save)
viz.animate_robot(save_path='trajectory_animation.gif', fps=10)
