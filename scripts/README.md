# Visualization Scripts

This directory contains Python scripts for visualizing optimal control problem (OCP) solutions and robot trajectories.

## Scripts

### visualize_trajectories.py

The main trajectory visualization tool with comprehensive plotting capabilities.

#### Features

- **Static Multi-Panel Plots**: Displays XY trajectory, position vs time, heading, and control inputs
- **Animated Visualization**: Shows robot moving along trajectory with real-time control display
- **Obstacle Display**: Visualizes circular obstacles loaded from JSON
- **Export Options**: Save plots as PNG and animations as GIF

#### Usage

```bash
# Show both static plot and animation (default)
python visualize_trajectories.py ocp_solution.json

# Show only static plot
python visualize_trajectories.py ocp_solution.json --static

# Show only animation
python visualize_trajectories.py ocp_solution.json --animate

# Save static plot to file
python visualize_trajectories.py ocp_solution.json --save-static trajectory.png

# Save animation to GIF
python visualize_trajectories.py ocp_solution.json --save-anim animation.gif --fps 10

# Custom obstacles file
python visualize_trajectories.py ocp_solution.json --obstacles path/to/obstacles.json

# Save without displaying
python visualize_trajectories.py ocp_solution.json --save-static plot.png --no-display
```

#### Arguments

- `json_file`: Path to trajectory JSON file (required)
- `--obstacles`: Path to obstacles JSON file (default: `src/cpp/robot/environment/obstacles.json`)
- `--static`: Show static multi-panel plot
- `--animate`: Show animation
- `--save-static PATH`: Save static plot to file
- `--save-anim PATH`: Save animation to GIF file
- `--fps N`: Animation frames per second (default: 10)
- `--no-display`: Don't display plots, only save them

### simple_visualize.py

A simplified example showing basic usage of the `TrajectoryVisualizer` class.

#### Features

- Demonstrates programmatic usage
- Loads trajectory and obstacle data
- Creates both static plot and animation

#### Usage

```bash
python simple_visualize.py
```

This will:
1. Load `ocp_solution.json` trajectory data
2. Load obstacles from `src/cpp/robot/environment/obstacles.json`
3. Display static plot and save to `trajectory_plot.png`
4. Display animation and save to `trajectory_animation.gif`

## TrajectoryVisualizer Class

The `TrajectoryVisualizer` class in `visualize_trajectories.py` can be used programmatically:

```python
from visualize_trajectories import TrajectoryVisualizer

# Initialize with trajectory file
viz = TrajectoryVisualizer('ocp_solution.json', 
                          obstacles_file='obstacles.json')

# Create static plot
viz.plot_static(save_path='output.png')

# Create animation
viz.animate_robot(save_path='output.gif', fps=10, show=True)
```

### Methods

- `__init__(json_file, obstacles_file=None)`: Load trajectory and obstacle data
- `plot_static(save_path=None)`: Create multi-panel static plot
- `animate_robot(save_path=None, fps=10, show=True)`: Create animated visualization

## Input JSON Format

### Trajectory JSON

The trajectory JSON file (e.g., `ocp_solution.json`) should have the format:

```json
{
  "metadata": {
    "num_intervals": 50,
    "sim_step": 0.2,
    "num_states": 3,
    "num_controls": 2
  },
  "states": {
    "x": [0.0, 0.1, ...],
    "y": [0.0, 0.05, ...],
    "theta": [0.0, 0.02, ...]
  },
  "controls": {
    "v": [1.0, 0.95, ...],
    "omega": [0.1, 0.12, ...]
  }
}
```

### Obstacles JSON

The obstacles JSON file should have the format:

```json
{
  "obstacle1": {
    "x": 2.0,
    "y": 2.0,
    "r": 1.0
  },
  "obstacle2": {
    "x": 7.0,
    "y": 6.0,
    "r": 2.0
  },
  "safety_margin": 1.5
}
```

## Dependencies

```bash
pip install numpy matplotlib
```

## Output Examples

- **Static Plots**: Multi-panel visualization showing trajectory path, state evolution, and control inputs
- **Animations**: GIF showing robot (blue circle with red heading arrow) moving along trajectory
- **Obstacle Visualization**: Red circles showing obstacle positions and sizes
