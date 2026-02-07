# Python Implementation

This directory contains the Python implementation of the robot dynamics model using CasADi.

## Overview

The Python implementation provides a lightweight version of the robot dynamics model, primarily for prototyping and testing. The main implementation is in C++, but this Python version offers:

- Easy experimentation with dynamics
- Quick prototyping of control algorithms
- Educational examples

## Structure

```
python/
├── main.py              # Example usage
└── robot/
    ├── __init__.py
    └── dynamics/
        ├── __init__.py
        └── model.py     # Robot dynamics model
```

## Components

### model.py - Robot Dynamics Model

The `Model` class encapsulates the robot's dynamic model for a differential drive system.

#### Class: `Model`

**Attributes:**
- `_parameters`: List of model parameters
- `_states`: Symbolic state variables `[x, y, theta]`
- `_controls`: Symbolic control variables `[v, omega]`
- `_discretized_dynamics`: CasADi function for discretized dynamics
- `_disc_step_size`: Symbolic discretization step size

**Methods:**

##### `__init__(parameters: List[float])`
Initializes the model with given parameters.

**Args:**
- `parameters`: List of floating-point model parameters

**Example:**
```python
params = [1.0, 2.0, 3.0]
bot = Model(params)
```

##### Properties

- `parameters`: Get model parameters (read-only)
- `states`: Get symbolic state variables (read-only)
- `controls`: Get symbolic control inputs (read-only)
- `discretized_dynamics`: Get discretized dynamics function (read-only)

#### Dynamics Model

The robot follows differential drive kinematics:

**Continuous Dynamics:**
```
ẋ = v * cos(θ)
ẏ = v * sin(θ)
θ̇ = ω
```

**States:**
- `x`: X position (m)
- `y`: Y position (m)
- `theta`: Orientation angle (rad)

**Controls:**
- `v`: Linear velocity (m/s)
- `omega`: Angular velocity (rad/s)

**Discretization:**
- Uses 4th-order Runge-Kutta method
- 5 integration steps per interval
- Implemented via `ca.simpleRK()`

## Usage Examples

### Basic Usage

```python
import casadi as ca
from robot.dynamics.model import Model

# Create model with parameters
params = [1.0, 2.0, 3.0]
bot = Model(params)

# Access symbolic representations
print(f"States: {bot.states}")
print(f"Controls: {bot.controls}")

# Use discretized dynamics
initial_state = ca.DM([0, 0, 0])      # [x, y, theta]
control_input = ca.DM([1.0, 0.5])     # [v, omega]
dt = 0.1                               # time step

next_state = bot.discretized_dynamics(initial_state, control_input, dt)
print(f"Next state: {next_state}")
```

### Simulating Multiple Steps

```python
import casadi as ca
from robot.dynamics.model import Model

bot = Model([1.0, 2.0, 3.0])

# Initial conditions
state = ca.DM([0, 0, 0])
control = ca.DM([1.0, 0.1])
dt = 0.1

# Simulate 10 steps
trajectory = [state]
for _ in range(10):
    state = bot.discretized_dynamics(state, control, dt)
    trajectory.append(state)

# Print trajectory
for i, s in enumerate(trajectory):
    print(f"Step {i}: x={float(s[0]):.3f}, y={float(s[1]):.3f}, theta={float(s[2]):.3f}")
```

## Running the Example

The `main.py` script demonstrates basic usage:

```bash
python src/python/main.py
```

**Output:**
```
States: [x, y, theta]
Controls: [v, omega]
Parameters: [1.0, 2.0, 3.0]
Next states: [x_next, y_next, theta_next]
```

## Dependencies

Install required packages:

```bash
pip install casadi
```

Or use the requirements file:

```bash
pip install -r requirements.txt
```

## Comparison with C++ Implementation

| Feature | Python | C++ |
|---------|--------|-----|
| Dynamics Model | ✓ | ✓ |
| Discretization | ✓ (RK4) | ✓ (RK4) |
| OCP Solver | ✗ | ✓ |
| MPC | ✗ | ✓ |
| Code Generation | ✗ | ✓ |
| Performance | Slower | Faster |

The Python implementation focuses on the dynamics model for prototyping, while the C++ implementation includes the full OCP solver, MPC, and performance optimizations.

## Further Development

To extend the Python implementation:

1. Add OCP formulation similar to C++ version
2. Implement MPC controller
3. Add parameter loading from JSON
4. Create visualization integration

See the C++ implementation (`src/cpp/`) for reference.
