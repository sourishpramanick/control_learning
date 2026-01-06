# Optimal Control Problem (OCP) Implementation

This directory contains an implementation of an Optimal Control Problem (OCP) solver for robot trajectory optimization using CasADi and IPOPT.

## Overview

The `Ocp` class provides a high-level interface for solving trajectory optimization problems. It uses:
- **CasADi**: A symbolic framework for automatic differentiation and numerical optimization
- **IPOPT**: An interior point optimizer for large-scale nonlinear optimization

## Files

- `include/Ocp.hpp`: Header file defining the `Ocp` class interface
- `Ocp.cpp`: Implementation of the `Ocp` class

## Features

### State and Control Variables
- **States**: `[x, y, theta]` - Robot position (x, y) and orientation (theta)
- **Controls**: `[v, omega]` - Linear velocity (v) and angular velocity (omega)

### Optimization Problem

The OCP solves the following trajectory optimization problem:

**Objective Function:**
```
minimize: terminal_cost + running_cost
where:
  terminal_cost = 100 * ||x_final - x_target||^2
  running_cost = sum over horizon of:
    - ||x_k - x_target||^2 (state deviation)
    - 0.1 * ||u_k||^2 (control effort)
```

**Subject to:**
- Dynamics constraints: `x_{k+1} = f(x_k, u_k)` (discretized robot dynamics)
- Initial state constraint: `x_0 = x_initial`
- Control bounds:
  - `min_velocity <= v <= max_velocity`
  - `min_angular_velocity <= omega <= max_angular_velocity`

## Usage

### Basic Example

```cpp
#include "robot/dynamics/include/Model.hpp"
#include "robot/ocp/include/Ocp.hpp"

// Create a robot model
robot::Model bot{};

// Create an OCP with 50-step horizon and 0.1s timestep
robot::Ocp ocp(bot, 50, 0.1);

// Set initial state [x, y, theta]
ocp.setInitialState({0.0, 0.0, 0.0});

// Set target state [x, y, theta]
ocp.setTarget({5.0, 3.0, 0.5});

// Solve the optimization problem
if (ocp.solve()) {
    // Get optimal trajectory
    auto states = ocp.getOptimalStates();
    auto controls = ocp.getOptimalControls();
    
    // Use the optimal trajectory...
}
```

### Constructor Parameters

```cpp
Ocp(const Model& model, int horizon = 50, double dt = 0.1)
```

- `model`: Reference to a `robot::Model` object containing robot dynamics
- `horizon`: Number of time steps in the planning horizon (default: 50)
- `dt`: Time step duration in seconds (default: 0.1)

### Methods

#### `void setInitialState(const std::vector<double>& initial)`
Sets the initial state for the robot.
- **Parameters**: `initial` - Vector of size 3 containing `[x, y, theta]`

#### `void setTarget(const std::vector<double>& target)`
Sets the target state for the robot to reach.
- **Parameters**: `target` - Vector of size 3 containing `[x, y, theta]`

#### `bool solve()`
Solves the optimal control problem using IPOPT.
- **Returns**: `true` if successful, `false` otherwise

#### `casadi::DM getOptimalStates() const`
Returns the optimal state trajectory.
- **Returns**: Matrix of size `3 x (horizon+1)` containing states at each time step

#### `casadi::DM getOptimalControls() const`
Returns the optimal control trajectory.
- **Returns**: Matrix of size `2 x horizon` containing controls at each time step

## Implementation Details

### Robot Dynamics

The robot follows a unicycle model with dynamics:
```
ẋ = v * cos(θ)
ẏ = v * sin(θ)
θ̇ = ω
```

These continuous dynamics are discretized using a 4th-order Runge-Kutta method with 5 integration steps per time interval.

### Solver Configuration

The IPOPT solver is configured with:
- Maximum iterations: 1000
- Tolerance: 1e-6
- Print level: 5 (verbose output for debugging)

### Cost Function Weights

- Terminal cost weight: 100 (strongly penalizes deviation from target at end)
- Running state cost weight: 1 (penalizes deviation from target along trajectory)
- Control effort weight: 0.1 (penalizes large control inputs)

These weights can be adjusted in the `setupOptimization()` method in `Ocp.cpp`.

## Example Output

See `ocp_example.cpp` for a complete working example that:
1. Creates a robot model
2. Sets up an OCP to drive from `[0, 0, 0]` to `[5, 3, 0.5]`
3. Solves the optimization problem
4. Displays the resulting trajectory

## Dependencies

- CasADi (>=3.5)
- IPOPT (typically bundled with CasADi)
- C++17 or later

## Notes

- The initial state defaults to `[0.0, 0.0, 0.0]` if not explicitly set
- Control bounds are read from the model's parameters file
- The solver may take several seconds to converge depending on the problem complexity
