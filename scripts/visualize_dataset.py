#!/usr/bin/env python3
"""
Visualize trajectories from the generated dataset (dataset.npz).

Allows browsing individual runs from the dataset and visualizing:
- Initial and target states
- Optimal trajectory (positions and controls)
- Obstacles
- Control inputs over time

Examples:
    # View random run
    python scripts/visualize_dataset.py
    
    # View specific run index
    python scripts/visualize_dataset.py --index 42
    
    # View multiple random runs
    python scripts/visualize_dataset.py --num-runs 5
    
    # Save plots without displaying
    python scripts/visualize_dataset.py --index 10 --save-dir plots/ --no-display
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow
from matplotlib.animation import FuncAnimation, PillowWriter


class DatasetVisualizer:
    """Visualize trajectories from the NPZ dataset"""
    
    def __init__(self, npz_path: str):
        """Load dataset from NPZ file
        
        Args:
            npz_path: Path to dataset.npz file
        """
        self.npz_path = Path(npz_path)
        if not self.npz_path.exists():
            raise FileNotFoundError(f"Dataset not found: {npz_path}")
        
        # Load dataset
        data = np.load(self.npz_path)
        self.inputs = data['inputs']   # Shape: (N, 22)
        self.outputs = data['outputs']  # Shape: (N, 248)
        data.close()
        
        self.n_samples = len(self.inputs)
        print(f"Loaded dataset: {self.n_samples} samples")
        
        # Load metadata
        meta_path = self.npz_path.parent / (self.npz_path.stem + "_meta.json")
        if meta_path.exists():
            with open(meta_path) as f:
                self.metadata = json.load(f)
            print(f"Loaded metadata from {meta_path}")
        else:
            print(f"Warning: Metadata file not found at {meta_path}")
            # Use defaults
            self.metadata = {
                "n_intervals": 50,
                "sim_step": 0.2,
                "obstacles": [],
                "floor_bounds": {"x": [0, 20], "y": [0, 20]}
            }
        
        self.n_intervals = self.metadata.get("n_intervals", 50)
        self.sim_step = self.metadata.get("sim_step", 0.2)
        self.obstacles = self.metadata.get("obstacles", [])
        self.floor_bounds = self.metadata.get("floor_bounds", {"x": [0, 20], "y": [0, 20]})
    
    def parse_sample(self, idx: int):
        """Parse a single sample from the dataset
        
        Args:
            idx: Sample index (0 to n_samples-1)
            
        Returns:
            dict with parsed data including states, controls, initial/target, obstacles
        """
        if idx < 0 or idx >= self.n_samples:
            raise ValueError(f"Index {idx} out of range [0, {self.n_samples-1}]")
        
        input_vec = self.inputs[idx]
        output_vec = self.outputs[idx]
        
        # Parse input (22 values)
        x0, y0, theta0 = input_vec[0:3]
        xT, yT, thetaT = input_vec[3:6]
        
        # Parse obstacles (15 values: 5 obstacles × 3 values each)
        obstacles_parsed = []
        for k in range(5):
            base = 6 + k * 3
            obstacles_parsed.append({
                "x": float(input_vec[base]),
                "y": float(input_vec[base + 1]),
                "r": float(input_vec[base + 2])
            })
        
        safety_margin = float(input_vec[21])
        
        # Parse output (248 values)
        # First 98 values: interleaved controls (v0, ω0, v1, ω1, ..., v48, ω48)
        # Next 150 values: interleaved states (x0, y0, θ0, x1, y1, θ1, ..., x49, y49, θ49)
        
        n_controls = 49  # v0...v48
        n_states = 50    # x0...x49
        
        # Extract controls (interleaved: v, ω, v, ω, ...)
        v = output_vec[0:2*n_controls:2]       # Every even index
        omega = output_vec[1:2*n_controls:2]   # Every odd index
        
        # Extract states (interleaved: x, y, θ, x, y, θ, ...)
        state_start = 2 * n_controls  # 98
        x = output_vec[state_start + 0::3]
        y = output_vec[state_start + 1::3]
        theta = output_vec[state_start + 2::3]
        
        return {
            "idx": idx,
            "initial": {"x": x0, "y": y0, "theta": theta0},
            "target": {"x": xT, "y": yT, "theta": thetaT},
            "obstacles": obstacles_parsed,
            "safety_margin": safety_margin,
            "states": {"x": x, "y": y, "theta": theta},
            "controls": {"v": v, "omega": omega},
            "time": np.arange(len(x)) * self.sim_step,
            "control_time": np.arange(len(v)) * self.sim_step,
        }
    
    def plot_trajectory(self, idx: int, save_path: str = None, show: bool = True):
        """Plot a single trajectory
        
        Args:
            idx: Sample index
            save_path: Optional path to save the plot
            show: Whether to display the plot
        """
        data = self.parse_sample(idx)
        
        fig = plt.figure(figsize=(16, 10), constrained_layout=True)
        gs = fig.add_gridspec(3, 3)
        
        # Main trajectory plot (larger, spans 2 rows)
        ax_traj = fig.add_subplot(gs[:2, :2])
        
        # Control plots
        ax_x = fig.add_subplot(gs[0, 2])
        ax_y = fig.add_subplot(gs[1, 2])
        ax_theta = fig.add_subplot(gs[2, 0])
        ax_v = fig.add_subplot(gs[2, 1])
        ax_omega = fig.add_subplot(gs[2, 2])
        
        fig.suptitle(f'Dataset Sample #{idx} / {self.n_samples}', 
                    fontsize=16, fontweight='bold')
        
        # ── Main trajectory plot ──
        # Plot obstacles
        for obs in data["obstacles"]:
            circle = Circle((obs['x'], obs['y']), obs['r'], 
                          color='red', alpha=0.3, zorder=1, label='Obstacle')
            ax_traj.add_patch(circle)
            ax_traj.plot(obs['x'], obs['y'], 'rx', markersize=8, markeredgewidth=2)
        
        # Plot trajectory
        ax_traj.plot(data["states"]["x"], data["states"]["y"], 
                    'b-', linewidth=2, label='Trajectory', zorder=3)
        
        # Plot start position
        init = data["initial"]
        ax_traj.plot(init["x"], init["y"], 'go', markersize=15, 
                    label='Start', zorder=4, markeredgecolor='darkgreen', markeredgewidth=2)
        # Start heading arrow
        arrow_len = 0.5
        ax_traj.arrow(init["x"], init["y"],
                     arrow_len * np.cos(init["theta"]),
                     arrow_len * np.sin(init["theta"]),
                     head_width=0.3, head_length=0.2, fc='green', ec='darkgreen',
                     linewidth=2, zorder=4)
        
        # Plot target position
        tgt = data["target"]
        ax_traj.plot(tgt["x"], tgt["y"], 'r*', markersize=20, 
                    label='Target', zorder=4, markeredgecolor='darkred', markeredgewidth=2)
        # Target heading arrow
        ax_traj.arrow(tgt["x"], tgt["y"],
                     arrow_len * np.cos(tgt["theta"]),
                     arrow_len * np.sin(tgt["theta"]),
                     head_width=0.3, head_length=0.2, fc='red', ec='darkred',
                     linewidth=2, zorder=4)
        
        ax_traj.set_xlabel('X (m)', fontsize=12)
        ax_traj.set_ylabel('Y (m)', fontsize=12)
        ax_traj.set_title('Optimal Trajectory', fontweight='bold', fontsize=14)
        ax_traj.grid(True, alpha=0.3)
        ax_traj.axis('equal')
        
        # Remove duplicate legend labels
        handles, labels = ax_traj.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax_traj.legend(by_label.values(), by_label.keys(), loc='best')
        
        # Add text annotations for initial and target states
        ax_traj.text(0.02, 0.98, 
                    f"Start: ({init['x']:.2f}, {init['y']:.2f}, {np.rad2deg(init['theta']):.1f}°)",
                    transform=ax_traj.transAxes, fontsize=10,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
        ax_traj.text(0.02, 0.90,
                    f"Goal:  ({tgt['x']:.2f}, {tgt['y']:.2f}, {np.rad2deg(tgt['theta']):.1f}°)",
                    transform=ax_traj.transAxes, fontsize=10,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7))
        
        # ── State plots ──
        time = data["time"]
        
        ax_x.plot(time, data["states"]["x"], 'b-', linewidth=2)
        ax_x.axhline(init["x"], color='green', linestyle='--', alpha=0.5, label='Start')
        ax_x.axhline(tgt["x"], color='red', linestyle='--', alpha=0.5, label='Target')
        ax_x.set_ylabel('X (m)', fontsize=10)
        ax_x.set_title('X Position', fontweight='bold')
        ax_x.grid(True, alpha=0.3)
        ax_x.legend(fontsize=8)
        
        ax_y.plot(time, data["states"]["y"], 'r-', linewidth=2)
        ax_y.axhline(init["y"], color='green', linestyle='--', alpha=0.5, label='Start')
        ax_y.axhline(tgt["y"], color='red', linestyle='--', alpha=0.5, label='Target')
        ax_y.set_ylabel('Y (m)', fontsize=10)
        ax_y.set_title('Y Position', fontweight='bold')
        ax_y.grid(True, alpha=0.3)
        ax_y.legend(fontsize=8)
        
        ax_theta.plot(time, np.rad2deg(data["states"]["theta"]), 'g-', linewidth=2)
        ax_theta.axhline(np.rad2deg(init["theta"]), color='green', linestyle='--', alpha=0.5)
        ax_theta.axhline(np.rad2deg(tgt["theta"]), color='red', linestyle='--', alpha=0.5)
        ax_theta.set_xlabel('Time (s)', fontsize=10)
        ax_theta.set_ylabel('θ (deg)', fontsize=10)
        ax_theta.set_title('Heading Angle', fontweight='bold')
        ax_theta.grid(True, alpha=0.3)
        
        # ── Control plots ──
        control_time = data["control_time"]
        
        ax_v.plot(control_time, data["controls"]["v"], 'm-', linewidth=2)
        ax_v.set_xlabel('Time (s)', fontsize=10)
        ax_v.set_ylabel('v (m/s)', fontsize=10)
        ax_v.set_title('Linear Velocity', fontweight='bold')
        ax_v.grid(True, alpha=0.3)
        
        ax_omega.plot(control_time, np.rad2deg(data["controls"]["omega"]), 'c-', linewidth=2)
        ax_omega.set_xlabel('Time (s)', fontsize=10)
        ax_omega.set_ylabel('ω (deg/s)', fontsize=10)
        ax_omega.set_title('Angular Velocity', fontweight='bold')
        ax_omega.grid(True, alpha=0.3)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Saved plot to: {save_path}")
        
        if show:
            plt.show()
        else:
            plt.close(fig)
    
    def plot_multiple(self, indices: list = None, num_random: int = 4, 
                     save_dir: str = None, show: bool = True):
        """Plot multiple trajectories in a grid
        
        Args:
            indices: List of specific indices to plot (if None, use random)
            num_random: Number of random samples to plot if indices is None
            save_dir: Directory to save individual plots
            show: Whether to display the plot
        """
        if indices is None:
            indices = np.random.choice(self.n_samples, 
                                      size=min(num_random, self.n_samples), 
                                      replace=False)
        
        n_plots = len(indices)
        cols = min(3, n_plots)
        rows = (n_plots + cols - 1) // cols
        
        fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 5*rows), 
                                 constrained_layout=True)
        if n_plots == 1:
            axes = np.array([[axes]])
        elif rows == 1:
            axes = axes.reshape(1, -1)
        elif cols == 1:
            axes = axes.reshape(-1, 1)
        
        fig.suptitle(f'Dataset Overview: {n_plots} Samples', 
                    fontsize=16, fontweight='bold')
        
        for plot_idx, sample_idx in enumerate(indices):
            row = plot_idx // cols
            col = plot_idx % cols
            ax = axes[row, col]
            
            data = self.parse_sample(sample_idx)
            
            # Plot obstacles
            for obs in data["obstacles"]:
                circle = Circle((obs['x'], obs['y']), obs['r'], 
                              color='red', alpha=0.3, zorder=1)
                ax.add_patch(circle)
            
            # Plot trajectory
            ax.plot(data["states"]["x"], data["states"]["y"], 
                   'b-', linewidth=1.5, alpha=0.8, zorder=3)
            
            # Start and goal
            init = data["initial"]
            tgt = data["target"]
            ax.plot(init["x"], init["y"], 'go', markersize=8, zorder=4)
            ax.plot(tgt["x"], tgt["y"], 'r*', markersize=12, zorder=4)
            
            ax.set_xlabel('X (m)', fontsize=9)
            ax.set_ylabel('Y (m)', fontsize=9)
            ax.set_title(f'Sample #{sample_idx}', fontweight='bold', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
        
        # Hide extra subplots
        for plot_idx in range(n_plots, rows * cols):
            row = plot_idx // cols
            col = plot_idx % cols
            axes[row, col].axis('off')
        
        if save_dir:
            save_path = Path(save_dir) / "dataset_overview.png"
            save_path.parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Saved overview to: {save_path}")
        
        if show:
            plt.show()
        else:
            plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description='Visualize trajectories from dataset.npz',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                                    # View random sample
  %(prog)s --index 42                         # View sample #42
  %(prog)s --num-runs 9                       # View 9 random samples
  %(prog)s --index 10 --save plots/run10.png  # Save specific run
  %(prog)s --overview --num-runs 12           # Grid overview of 12 samples
        """
    )
    
    parser.add_argument('--dataset', type=str, default='data/dataset.npz',
                       help='Path to dataset.npz file (default: data/dataset.npz)')
    parser.add_argument('--index', type=int, 
                       help='Specific sample index to visualize')
    parser.add_argument('--num-runs', type=int, default=1,
                       help='Number of random samples to visualize (default: 1)')
    parser.add_argument('--overview', action='store_true',
                       help='Show grid overview of multiple samples')
    parser.add_argument('--save', type=str,
                       help='Path to save the plot (for single trajectory)')
    parser.add_argument('--save-dir', type=str,
                       help='Directory to save plots (for multiple trajectories)')
    parser.add_argument('--no-display', action='store_true',
                       help='Do not display plots, only save them')
    parser.add_argument('--seed', type=int,
                       help='Random seed for selecting samples')
    
    args = parser.parse_args()
    
    # Set random seed if provided
    if args.seed is not None:
        np.random.seed(args.seed)
    
    # Load visualizer
    viz = DatasetVisualizer(args.dataset)
    
    show_plots = not args.no_display
    
    if args.overview:
        # Show overview grid
        viz.plot_multiple(num_random=args.num_runs, 
                         save_dir=args.save_dir,
                         show=show_plots)
    elif args.index is not None:
        # Show specific index
        viz.plot_trajectory(args.index, save_path=args.save, show=show_plots)
    elif args.num_runs > 1:
        # Show multiple random samples
        indices = np.random.choice(viz.n_samples, size=args.num_runs, replace=False)
        for idx in indices:
            save_path = None
            if args.save_dir:
                save_path = Path(args.save_dir) / f"sample_{idx:05d}.png"
                save_path.parent.mkdir(parents=True, exist_ok=True)
            viz.plot_trajectory(idx, save_path=save_path, show=show_plots)
    else:
        # Show one random sample
        idx = np.random.randint(0, viz.n_samples)
        print(f"Showing random sample #{idx}")
        viz.plot_trajectory(idx, save_path=args.save, show=show_plots)


if __name__ == '__main__':
    main()
