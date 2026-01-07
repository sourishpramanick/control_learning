#!/usr/bin/env python3
"""
Visualize OCP Solution Trajectories
Reads JSON trajectory data and creates animated plots
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyArrow, Circle, Rectangle
import argparse
from pathlib import Path


class TrajectoryVisualizer:
    """Visualize robot trajectories from OCP solution"""
    
    def __init__(self, json_file):
        """Load trajectory data from JSON file"""
        with open(json_file, 'r') as f:
            data = json.load(f)
        
        self.metadata = data['metadata']
        self.states = data['states']
        self.controls = data['controls']
        
        self.x = np.array(self.states['x'])
        self.y = np.array(self.states['y'])
        self.theta = np.array(self.states['theta'])
        self.v = np.array(self.controls['v'])
        self.omega = np.array(self.controls['omega'])
        
        self.num_intervals = self.metadata['num_intervals']
        self.sim_step = self.metadata['sim_step']
        
        # Time vector
        self.time = np.arange(len(self.x)) * self.sim_step
        self.control_time = np.arange(len(self.v)) * self.sim_step
        
    def plot_static(self, save_path=None):
        """Create static multi-panel plot"""
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle('OCP Solution Trajectories', fontsize=16, fontweight='bold')
        
        # XY trajectory
        ax = axes[0, 0]
        ax.plot(self.x, self.y, 'b-', linewidth=2, label='Path')
        ax.plot(self.x[0], self.y[0], 'go', markersize=12, label='Start')
        ax.plot(self.x[-1], self.y[-1], 'r*', markersize=15, label='Goal')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('XY Trajectory', fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
        
        # X vs time
        ax = axes[0, 1]
        ax.plot(self.time, self.x, 'b-', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('X (m)', fontsize=12)
        ax.set_title('X Position', fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Y vs time
        ax = axes[0, 2]
        ax.plot(self.time, self.y, 'r-', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Y Position', fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Theta vs time
        ax = axes[1, 0]
        ax.plot(self.time, np.rad2deg(self.theta), 'g-', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Theta (deg)', fontsize=12)
        ax.set_title('Heading Angle', fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Velocity vs time
        ax = axes[1, 1]
        ax.plot(self.control_time, self.v, 'm-', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('v (m/s)', fontsize=12)
        ax.set_title('Linear Velocity', fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Angular velocity vs time
        ax = axes[1, 2]
        ax.plot(self.control_time, np.rad2deg(self.omega), 'c-', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('ω (deg/s)', fontsize=12)
        ax.set_title('Angular Velocity', fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Static plot saved to: {save_path}")
        
        plt.show()
        
    def animate_robot(self, save_path=None, fps=10, show=True):
        """Create animated visualization of robot moving along trajectory
        
        Args:
            save_path: Path to save GIF (optional)
            fps: Frames per second for animation
            show: If True, display animation window. If False, only save.
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('Robot Trajectory Animation', fontsize=16, fontweight='bold')
        
        # Left plot: XY trajectory with robot
        ax1.plot(self.x, self.y, 'b--', linewidth=1, alpha=0.5, label='Path')
        ax1.plot(self.x[0], self.y[0], 'go', markersize=12, label='Start')
        ax1.plot(self.x[-1], self.y[-1], 'r*', markersize=15, label='Goal')
        ax1.set_xlabel('X (m)', fontsize=12)
        ax1.set_ylabel('Y (m)', fontsize=12)
        ax1.set_title('Robot Path', fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right')
        ax1.axis('equal')
        
        # Robot representation (circle + arrow for heading)
        robot_size = 0.2
        robot_circle = Circle((self.x[0], self.y[0]), robot_size, 
                             color='blue', alpha=0.6, zorder=5)
        ax1.add_patch(robot_circle)
        
        # Heading arrow (initialize as None, will be created in animate)
        arrow_length = 0.3
        heading_arrow = [None]  # Use list to make it mutable in nested function
        
        # Trajectory trace
        trace_line, = ax1.plot([], [], 'b-', linewidth=2, label='Traveled')
        
        # Time text
        time_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes,
                           fontsize=12, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Right plot: Controls over time
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Control Input', fontsize=12)
        ax2.set_title('Control Inputs', fontweight='bold')
        ax2.grid(True, alpha=0.3)
        
        # Plot full control trajectories
        ax2.plot(self.control_time, self.v, 'g--', linewidth=1, alpha=0.5, label='v (m/s)')
        ax2_twin = ax2.twinx()
        ax2_twin.plot(self.control_time, np.rad2deg(self.omega), 'r--', 
                     linewidth=1, alpha=0.5, label='ω (deg/s)')
        
        ax2.set_ylabel('v (m/s)', color='g', fontsize=12)
        ax2_twin.set_ylabel('ω (deg/s)', color='r', fontsize=12)
        
        # Current control markers
        v_point, = ax2.plot([], [], 'go', markersize=10, label='Current v')
        omega_point, = ax2_twin.plot([], [], 'ro', markersize=10, label='Current ω')
        
        # Legends
        lines1, labels1 = ax2.get_legend_handles_labels()
        lines2, labels2 = ax2_twin.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        
        def init():
            """Initialize animation"""
            trace_line.set_data([], [])
            time_text.set_text('')
            v_point.set_data([], [])
            omega_point.set_data([], [])
            return trace_line, robot_circle, time_text, v_point, omega_point
        
        def animate(frame):
            """Update animation for each frame"""
            # Update robot position
            robot_circle.center = (self.x[frame], self.y[frame])
            
            # Remove old arrow if it exists
            if heading_arrow[0] is not None:
                heading_arrow[0].remove()
            
            # Create new heading arrow
            heading_arrow[0] = FancyArrow(
                self.x[frame], self.y[frame],
                arrow_length * np.cos(self.theta[frame]),
                arrow_length * np.sin(self.theta[frame]),
                width=0.1, head_width=0.2, head_length=0.15,
                color='red', zorder=6
            )
            ax1.add_patch(heading_arrow[0])
            
            # Update trajectory trace
            trace_line.set_data(self.x[:frame+1], self.y[:frame+1])
            
            # Update time text
            time_text.set_text(f'Time: {self.time[frame]:.2f} s')
            
            # Update control points (if within control array bounds)
            if frame < len(self.v):
                v_point.set_data([self.control_time[frame]], [self.v[frame]])
                omega_point.set_data([self.control_time[frame]], 
                                    [np.rad2deg(self.omega[frame])])
            
            return trace_line, robot_circle, heading_arrow[0], time_text, v_point, omega_point
        
        # Create animation
        anim = FuncAnimation(
            fig, animate, init_func=init,
            frames=len(self.x), interval=1000/fps, blit=False, repeat=True
        )
        
        if save_path:
            print(f"Saving animation to {save_path}... (this may take a minute)")
            writer = PillowWriter(fps=fps)
            anim.save(save_path, writer=writer)
            print(f"Animation saved to: {save_path}")
        
        if show:
            plt.show()
        else:
            plt.close(fig)
        
        return anim


def main():
    parser = argparse.ArgumentParser(description='Visualize OCP trajectories')
    parser.add_argument('json_file', type=str, help='Path to trajectory JSON file')
    parser.add_argument('--static', action='store_true', help='Show static plot')
    parser.add_argument('--animate', action='store_true', help='Show animation')
    parser.add_argument('--save-static', type=str, help='Save static plot to file')
    parser.add_argument('--save-anim', type=str, help='Save animation to GIF')
    parser.add_argument('--fps', type=int, default=10, help='Animation FPS (default: 10)')
    parser.add_argument('--no-display', action='store_true', 
                       help='Do not display plots (only save them)')
    
    args = parser.parse_args()
    
    # Load trajectories
    viz = TrajectoryVisualizer(args.json_file)
    
    # Determine if we should show plots
    show_plots = not args.no_display
    
    # Show static plot
    if args.static or args.save_static:
        viz.plot_static(save_path=args.save_static)
    
    # Show animation
    if args.animate or args.save_anim:
        viz.animate_robot(save_path=args.save_anim, fps=args.fps, show=show_plots)
    
    # If no flags, show both
    if not any([args.static, args.animate, args.save_static, args.save_anim]):
        print("Showing static plot...")
        viz.plot_static()
        print("\nShowing animation...")
        viz.animate_robot(show=show_plots)


if __name__ == '__main__':
    main()
