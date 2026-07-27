#!/usr/bin/env python3
"""
Simple example: Visualize dataset trajectories interactively

This script demonstrates basic usage of the DatasetVisualizer class.
Modify the parameters below to explore your dataset.
"""

from visualize_dataset import DatasetVisualizer
import numpy as np

# ── Configuration ──────────────────────────────────────────────────────────────
DATASET_PATH = 'data/dataset.npz'
RANDOM_SEED = 42  # For reproducible random selection

# ── Load dataset ───────────────────────────────────────────────────────────────
viz = DatasetVisualizer(DATASET_PATH)

print(f"\nDataset contains {viz.n_samples} successful OCP solutions")
print(f"Each input has {viz.metadata['input_size']} features")
print(f"Each output has {viz.metadata['output_size']} values")
print(f"Success rate during generation: {viz.metadata['success_rate']:.1%}\n")

# ── Example 1: Visualize a specific sample ────────────────────────────────────
print("Example 1: Visualizing sample #42...")
viz.plot_trajectory(42, save_path='demo_plots/sample_42.png')

# ── Example 2: Visualize a random sample ──────────────────────────────────────
print("\nExample 2: Visualizing a random sample...")
np.random.seed(RANDOM_SEED)
random_idx = np.random.randint(0, viz.n_samples)
print(f"Selected sample #{random_idx}")
viz.plot_trajectory(random_idx)

# ── Example 3: Overview of multiple samples ───────────────────────────────────
print("\nExample 3: Creating overview grid of 9 samples...")
np.random.seed(RANDOM_SEED)
viz.plot_multiple(num_random=9, save_dir='demo_plots/')

# ── Example 4: Programmatic access to data ────────────────────────────────────
print("\nExample 4: Accessing data programmatically...")
sample = viz.parse_sample(0)

print(f"  Initial state: x={sample['initial']['x']:.2f}, "
      f"y={sample['initial']['y']:.2f}, "
      f"θ={np.rad2deg(sample['initial']['theta']):.1f}°")
print(f"  Target state:  x={sample['target']['x']:.2f}, "
      f"y={sample['target']['y']:.2f}, "
      f"θ={np.rad2deg(sample['target']['theta']):.1f}°")
print(f"  Trajectory length: {len(sample['states']['x'])} timesteps")
print(f"  Max velocity: {np.max(sample['controls']['v']):.2f} m/s")
print(f"  Max angular velocity: {np.rad2deg(np.max(np.abs(sample['controls']['omega']))):.1f} deg/s")

# ── Example 5: Analyze dataset statistics ─────────────────────────────────────
print("\nExample 5: Dataset statistics...")

# Sample a subset for analysis
n_analyze = min(100, viz.n_samples)
max_velocities = []
path_lengths = []

for idx in range(n_analyze):
    sample = viz.parse_sample(idx)
    max_velocities.append(np.max(sample['controls']['v']))
    
    # Compute path length
    x_diff = np.diff(sample['states']['x'])
    y_diff = np.diff(sample['states']['y'])
    path_length = np.sum(np.sqrt(x_diff**2 + y_diff**2))
    path_lengths.append(path_length)

print(f"  Analyzed {n_analyze} samples:")
print(f"  Average max velocity: {np.mean(max_velocities):.2f} m/s")
print(f"  Average path length: {np.mean(path_lengths):.2f} m")
print(f"  Shortest path: {np.min(path_lengths):.2f} m")
print(f"  Longest path: {np.max(path_lengths):.2f} m")

print("\nDone! Check demo_plots/ for saved figures.")
