# Robot Dynamics

This directory contains the implementation of the robot dynamics model using CasADi.

## Overview

The `Model` class represents a differential drive robot's kinematics and provides both continuous and discretized dynamics functions for use in optimal control and simulation.

## Files

- `include/Model.hpp` - Model class header file
- `Model.cpp` - Model class implementation

## Model Class

### Description

The `Model` class encapsulates the robot's dynamic model, including:
- Symbolic state and control variables
- Robot parameters loaded from JSON
- Continuous dynamics (differential equations)
- Discretized dynamics using Runge-Kutta integration

### Constructor

```cpp
Model(const std::string& parametersPath = PARAMETERS_FILE_PATH);
```

**Parameters:**
- `parametersPath`: Path to JSON file containing robot parameters (default set via CMake)

**Throws:**
- `std::runtime_error` if parameters file not found or invalid

### Robot Kinematics

The robot follows differential drive kinematics:

**States (3D):**
- `x`: Position in X direction (m)
- `y`: Position in Y direction (m)
- `theta`: Heading angle (rad)

**Controls (2D):**
- `v`: Linear velocity (m/s)
- `omega`: Angular velocity (rad/s)

**Continuous Dynamics:**
```
ẋ = v * cos(θ)
ẏ = v * sin(θ)
θ̇ = ω
```

### Parameters

Loaded from `src/cpp/robot/data/parameters.json`:

```json
{
  "parameters": {
    "wheel_base": 0.5,
    "max_velocity": 2.0,
    "min_velocity": -1.0,
    "max_angular_velocity": 1.0,
    "min_angular_velocity": -1.0
  },
  "dynamics": {
    "discretization_step_size": 0.1
  }
}
```

### Public Methods

#### Getters

```cpp
const casadi::SX& getStates() const;
```
Returns symbolic state variables `[x, y, theta]`.

```cpp
const casadi::SX& getControls() const;
```
Returns symbolic control variables `[v, omega]`.

```cpp
const std::map<std::string, double>& getParameters() const;
```
Returns map of parameter names to values.

```cpp
const casadi::Function& getContinuousDynamics() const;
```
Returns continuous dynamics function: `f(states, controls) -> states_dot`.

```cpp
const casadi::Function& getDiscretizedDynamics() const;
```
Returns discretized dynamics function: `f(states, controls, dt) -> next_states`.

### Usage Example

```cpp
#include "robot/dynamics/include/Model.hpp"

// Create model (loads default parameters from PARAMETERS_FILE_PATH)
robot::Model bot{};

// Access states and controls
auto states = bot.getStates();      // [x, y, theta]
auto controls = bot.getControls();  // [v, omega]

// Get parameters
auto params = bot.getParameters();
double max_v = params.at("max_velocity");

// Use discretized dynamics
std::vector<double> init_state = {0.0, 0.0, 0.0};  // x=0, y=0, theta=0
std::vector<double> control = {1.0, 0.1};          // v=1.0, omega=0.1
double dt = 0.1;

auto next_state = bot.getDiscretizedDynamics()(casadi::DMVector{
    casadi::DM(init_state),
    casadi::DM(control),
    casadi::DM(dt)
})[0];

std::cout << "Next state: " << next_state << std::endl;
```

## Implementation Details

### Discretization

The continuous dynamics are discretized using **4th-order Runge-Kutta (RK4)** integration:

- **Method**: `casadi::simpleRK()`
- **Order**: 4 (RK4)
- **Steps per interval**: 5 (for accuracy)

This provides a numerical integration scheme that:
- Is 4th-order accurate
- Handles nonlinear dynamics
- Integrates over variable time steps

### Private Methods

```cpp
void computeContinuousDynamics();
```
Creates the continuous dynamics CasADi function from the kinematic equations.

```cpp
void discretizeContinuousDynamics();
```
Applies Runge-Kutta discretization to the continuous dynamics.

### Member Variables

```cpp
casadi::SX m_states;                          // Symbolic states [x, y, theta]
casadi::SX m_controls;                        // Symbolic controls [v, omega]
casadi::SX m_discStepSize;                   // Symbolic discretization step
std::map<std::string, double> m_parameters;  // Robot parameters
casadi::Function m_continuousDynamics;       // Continuous dynamics function
casadi::Function m_discretizedDynamics;      // Discretized dynamics function
```

## Dependencies

- **CasADi**: For symbolic computation and automatic differentiation
- **nlohmann/json**: For JSON parameter parsing
- **C++17**: Uses `std::filesystem` for path handling

## Design Considerations

### Why CasADi?

CasADi provides:
1. Symbolic differentiation (automatic Jacobians/Hessians)
2. Efficient code generation
3. Integration with NLP solvers (IPOPT)
4. Built-in integrators (RK4, etc.)

### Why RK4 Discretization?

- **Accuracy**: 4th-order method provides good accuracy
- **Stability**: More stable than explicit Euler
- **Multiple steps**: 5 sub-steps per interval increases accuracy
- **Nonlinear handling**: Works well with nonlinear dynamics

### Parameter Loading

Parameters are loaded from JSON to allow easy tuning without recompilation. The parameter structure supports nested parameters for future extensibility.

## Future Enhancements

Potential improvements:
- Add dynamics model variants (e.g., Ackermann steering)
- Support time-varying parameters
- Add state constraints to model
- Implement higher-order integrators
- Add dynamics linearization methods

## See Also

- [OCP Documentation](../ocp/README.md) - How the model is used in optimization
- [Parameters File](../data/parameters.json) - Parameter configuration
- Main [C++ README](../README.md) - Overall C++ implementation
