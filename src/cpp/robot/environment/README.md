# Environment Configuration

This directory contains JSON configuration files that define the environment for optimal control and trajectory planning.

## Overview

The environment configuration includes:
- Robot's initial state (starting position and orientation)
- Target state (goal position and orientation)
- Obstacle positions and sizes

These files allow easy modification of scenarios without recompiling the code.

## Configuration Files

### initial_state.json

Defines the robot's starting position and orientation.

**Format:**
```json
{
  "x": 0.0,
  "y": 0.0,
  "theta": 0.0
}
```

**Fields:**
- `x`: Initial X position in meters
- `y`: Initial Y position in meters
- `theta`: Initial heading angle in radians

**Example:**
```json
{
  "x": 1.5,
  "y": 2.0,
  "theta": 0.785
}
```
Starts robot at position (1.5, 2.0) with heading π/4 radians (45°).

### target.json

Defines the goal position and desired orientation.

**Format:**
```json
{
  "x": 20.0,
  "y": 20.0,
  "theta": 0.0
}
```

**Fields:**
- `x`: Target X position in meters
- `y`: Target Y position in meters
- `theta`: Target heading angle in radians

**Example:**
```json
{
  "x": 10.0,
  "y": 8.0,
  "theta": 1.571
}
```
Sets goal at (10, 8) with heading π/2 radians (90°, facing up).

### obstacles.json

Defines circular obstacles in the environment and safety margin.

**Format:**
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

**Obstacle Fields:**
- `x`: Obstacle center X position (m)
- `y`: Obstacle center Y position (m)
- `r`: Obstacle radius (m)

**Special Fields:**
- `safety_margin`: Additional clearance around obstacles (m)

**Notes:**
- Obstacle names (e.g., "obstacle1") are arbitrary and used only for identification
- The key "safety_margin" is reserved and will not be treated as an obstacle
- Any number of obstacles can be defined

## Usage

### Loading in C++

The configuration files are loaded automatically by `Optimizer::Optimize()` and `Optimizer::MPC()`:

```cpp
// Load initial state
utilities::json initStateJson = utilities::loadJson(
    "src/cpp/robot/environment/initial_state.json"
);
std::vector<double> initState{
    initStateJson["x"].get<double>(),
    initStateJson["y"].get<double>(),
    initStateJson["theta"].get<double>()
};

// Load target
utilities::json targetJson = utilities::loadJson(
    "src/cpp/robot/environment/target.json"
);
std::vector<double> target{
    targetJson["x"].get<double>(),
    targetJson["y"].get<double>(),
    targetJson["theta"].get<double>()
};

// Load obstacles
utilities::json obstacleJson = utilities::loadJson(
    "src/cpp/robot/environment/obstacles.json"
);
double safetyMargin = obstacleJson.value("safety_margin", 0.0);

std::vector<std::vector<double>> obstacles;
for (const auto& [key, value] : obstacleJson.items()) {
    if (key == "safety_margin") continue;
    obstacles.push_back({
        value["x"].get<double>(),
        value["y"].get<double>(),
        value["r"].get<double>()
    });
}
```

### Loading in Python (Visualization)

The visualization scripts automatically load obstacles for display:

```python
from visualize_trajectories import TrajectoryVisualizer

viz = TrajectoryVisualizer(
    'ocp_solution.json',
    obstacles_file='src/cpp/robot/environment/obstacles.json'
)
```

## Coordinate System

- **Origin**: (0, 0) is at the bottom-left
- **X-axis**: Positive to the right
- **Y-axis**: Positive upward
- **Theta**: Counter-clockwise from positive X-axis
  - 0 rad: Facing right (→)
  - π/2 rad: Facing up (↑)
  - π rad: Facing left (←)
  - 3π/2 rad: Facing down (↓)

## Example Scenarios

### Simple Straight Line

**initial_state.json:**
```json
{
  "x": 0.0,
  "y": 0.0,
  "theta": 0.0
}
```

**target.json:**
```json
{
  "x": 10.0,
  "y": 0.0,
  "theta": 0.0
}
```

**obstacles.json:**
```json
{
  "safety_margin": 0.5
}
```

Robot moves from (0,0) to (10,0) in a straight line.

### Single Obstacle Avoidance

**initial_state.json:**
```json
{
  "x": 0.0,
  "y": 0.0,
  "theta": 0.0
}
```

**target.json:**
```json
{
  "x": 10.0,
  "y": 0.0,
  "theta": 0.0
}
```

**obstacles.json:**
```json
{
  "obstacle1": {
    "x": 5.0,
    "y": 0.0,
    "r": 2.0
  },
  "safety_margin": 1.0
}
```

Robot must navigate around obstacle at (5,0) to reach goal.

### Maze Navigation

**initial_state.json:**
```json
{
  "x": 0.0,
  "y": 0.0,
  "theta": 0.0
}
```

**target.json:**
```json
{
  "x": 20.0,
  "y": 20.0,
  "theta": 0.0
}
```

**obstacles.json:**
```json
{
  "obstacle1": {
    "x": 5.0,
    "y": 5.0,
    "r": 2.0
  },
  "obstacle2": {
    "x": 10.0,
    "y": 10.0,
    "r": 3.0
  },
  "obstacle3": {
    "x": 15.0,
    "y": 7.0,
    "r": 2.5
  },
  "obstacle4": {
    "x": 8.0,
    "y": 15.0,
    "r": 2.0
  },
  "safety_margin": 1.5
}
```

Robot navigates through multiple obstacles.

## Constraint Formulation

### Obstacle Avoidance

For each obstacle at `(x_obs, y_obs)` with radius `r_obs`:

```
distance² = (x - x_obs)² + (y - y_obs)²
constraint: distance² ≥ (r_obs + safety_margin)²
```

This ensures the robot maintains at least `safety_margin` distance from obstacle boundary.

### Safety Margin

The safety margin serves multiple purposes:
1. **Collision buffer**: Accounts for robot size and uncertainty
2. **Smoother paths**: Prevents tight maneuvering around obstacles
3. **Numerical stability**: Prevents constraints from becoming active at boundaries

**Typical values:**
- Small margin (0.1-0.5m): Tight spaces, efficient paths
- Medium margin (0.5-1.5m): Balanced safety and efficiency
- Large margin (1.5-3.0m): High safety, conservative paths

## Validation

### Valid Configuration Checks

1. **Initial state not inside obstacle:**
   ```
   ∀ obstacles: (x_init - x_obs)² + (y_init - y_obs)² > (r_obs + margin)²
   ```

2. **Target not inside obstacle:**
   ```
   ∀ obstacles: (x_target - x_obs)² + (y_target - y_obs)² > (r_obs + margin)²
   ```

3. **Path exists:**
   - There must be a collision-free path from start to goal
   - Obstacles shouldn't completely block all paths

### Testing Configurations

Before running optimization, validate:
```bash
# Check if files are well-formed JSON
cat initial_state.json | python -m json.tool
cat target.json | python -m json.tool
cat obstacles.json | python -m json.tool
```

## Troubleshooting

**Solver fails (infeasible):**
- Check if obstacles block all paths
- Reduce safety margin
- Move initial/target away from obstacles

**Robot hits obstacles in simulation:**
- Increase safety margin
- Check discretization timestep is small enough
- Verify dynamics model is accurate

**Unexpected paths:**
- Check coordinate system (theta in radians!)
- Verify obstacle positions are correct
- Check if cost function is appropriate

## See Also

- [OCP README](../ocp/README.md) - How environment is used in optimization
- [Visualization Scripts](../../../../scripts/README.md) - Visualizing obstacles
- [Main C++ README](../../README.md) - Overall implementation
