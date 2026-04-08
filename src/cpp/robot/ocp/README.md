# Optimal Control Problem (OCP)

This directory contains the implementation of the Optimal Control Problem formulation, solver, and optimization routines.

## Overview

The OCP module formulates and solves trajectory optimization problems for the robot, including:
- Trajectory planning from start to goal
- Obstacle avoidance constraints
- Control effort minimization
- Model Predictive Control (MPC) implementation

## Files

- `Ocp.hpp` / `Ocp.cpp` - OCP class for problem formulation and solving
- `Optimizer.hpp` / `Optimizer.cpp` - High-level optimization interface

## Ocp Class

### Description

The `Ocp` class formulates and solves optimal control problems using CasADi and IPOPT.

### Constructor

```cpp
Ocp(int N, robot::Model&& bot, double safetyMargin, double simStep);
```

**Parameters:**
- `N`: Number of discretization intervals (nodes in trajectory)
- `bot`: Robot model (moved into OCP)
- `safetyMargin`: Additional safety margin around obstacles (m)
- `simStep`: Simulation time step (s)

### Public Methods

#### setupOcp

```cpp
void setupOcp(std::vector<std::vector<double>>&& obstacles);
```

Formulates the NLP problem with decision variables, constraints, and cost function.

**Parameters:**
- `obstacles`: Vector of obstacles, each defined as `[x, y, radius]`

**Creates:**
- Decision variables (states and controls)
- Equality constraints (initial state, dynamics continuity)
- Inequality constraints (control bounds, obstacle avoidance)
- Quadratic cost function

#### solveOcp

```cpp
int solveOcp(const std::vector<double>& initState, 
             const std::vector<double>& targetState);
```

Solves the OCP for given initial and target states.

**Parameters:**
- `initState`: Initial state `[x, y, theta]`
- `targetState`: Target state `[x, y, theta]`

**Returns:**
- `0`: Success
- `1`: Failure

#### extractSolution

```cpp
void extractSolution();
```

Extracts state and control trajectories from the optimization solution.

**Populates:**
- `m_xTraj`, `m_yTraj`, `m_thetaTraj`: State trajectories
- `m_vTraj`, `m_omegaTraj`: Control trajectories

#### saveTrajectoriesToJson

```cpp
void saveTrajectoriesToJson(const std::string& filename) const;
```

Saves the solution trajectories to a JSON file.

**Parameters:**
- `filename`: Output JSON file path

**Output format:**
```json
{
  "metadata": {
    "num_intervals": 50,
    "sim_step": 0.2,
    "num_states": 3,
    "num_controls": 2
  },
  "states": {
    "x": [...],
    "y": [...],
    "theta": [...]
  },
  "controls": {
    "v": [...],
    "omega": [...]
  }
}
```

#### createInitialGuess

```cpp
void createInitialGuess();
```

Creates an initial guess for the optimization (currently all ones).

#### generateCode

```cpp
void generateCode();
```

Generates standalone C code for the NLP solver (saved to `src/c/robot/generated_code/`).

#### Getters

```cpp
const std::vector<double>& getXTrajectory() const;
const std::vector<double>& getYTrajectory() const;
const std::vector<double>& getThetaTrajectory() const;
const std::vector<double>& getVTrajectory() const;
const std::vector<double>& getOmegaTrajectory() const;
int getNumStates() const;
int getNumControls() const;
int getNumIntervals() const;
double getSimStep() const;
```

## Optimizer Class

### Description

The `Optimizer` class provides high-level interfaces for optimization tasks, handling configuration loading and orchestration.

### Static Methods

#### Optimize

```cpp
static void Optimize();
```

Performs single trajectory optimization:
1. Loads initial state, target, and obstacles from JSON
2. Creates robot model
3. Sets up and solves OCP
4. Saves solution to `ocp_solution.json`

**Configuration Files:**
- `src/cpp/robot/environment/initial_state.json`
- `src/cpp/robot/environment/target.json`
- `src/cpp/robot/environment/obstacles.json`

#### MPC

```cpp
static void MPC();
```

Runs Model Predictive Control loop:
1. Loads initial state and target
2. Repeatedly:
   - Solves OCP for current state
   - Applies first control input
   - Updates state using dynamics
   - Checks if target reached
3. Saves trajectory to `robot_sim.json`

**Features:**
- Receding horizon control
- Real-time replanning
- Target proximity detection
- Trajectory logging

## Problem Formulation

### Decision Variables

For N intervals:
- **States**: `X = [x₀, y₀, θ₀, x₁, y₁, θ₁, ..., xₙ, yₙ, θₙ]` (3N variables)
- **Controls**: `U = [v₀, ω₀, v₁, ω₁, ..., vₙ₋₁, ωₙ₋₁]` (2(N-1) variables)

Total: `5N - 2` decision variables

### Cost Function

```
J = Σᵢ₌₀ᴺ⁻¹ (||xᵢ - x_target||² + ||uᵢ||²) + ||xₙ - x_target||²_Qf
```

Where:
- Stage cost: State deviation + control effort
- Terminal cost: Weighted final state deviation
- `Qf = diag(100, 100, 100)`: Terminal weight matrix

### Constraints

#### 1. Initial State Constraint
```
x₀ = x_init
```

#### 2. Dynamics Constraints
For each interval i:
```
xᵢ₊₁ = f(xᵢ, uᵢ, dt)
```
Where `f` is the discretized dynamics from the Model class.

#### 3. Control Bounds
```
v_min ≤ vᵢ ≤ v_max
ω_min ≤ ωᵢ ≤ ω_max
```

Values from `parameters.json`.

#### 4. Obstacle Avoidance
For each obstacle j at position `(xⱼ, yⱼ)` with radius `rⱼ`:
```
(xᵢ - xⱼ)² + (yᵢ - yⱼ)² ≥ (rⱼ + safety_margin)²
```

Reformulated as:
```
-(xᵢ - xⱼ)² - (yᵢ - yⱼ)² + rⱼ² ≤ -safety_margin
```

## Usage Examples

### Single Optimization

```cpp
#include "robot/ocp/Optimizer.hpp"

int main() {
    Ocp::Optimizer::Optimize();
    return 0;
}
```

### Custom Optimization

```cpp
#include "robot/ocp/Ocp.hpp"
#include "robot/dynamics/include/Model.hpp"

// Create model
robot::Model bot{};

// Create OCP (50 intervals, 0.2s timestep, 1.5m safety margin)
Ocp::Ocp ocp(50, std::move(bot), 1.5, 0.2);

// Define obstacles
std::vector<std::vector<double>> obstacles = {
    {2.0, 2.0, 1.0},   // x, y, radius
    {7.0, 6.0, 2.0}
};

// Setup and solve
ocp.setupOcp(std::move(obstacles));
ocp.createInitialGuess();

std::vector<double> init = {0.0, 0.0, 0.0};
std::vector<double> target = {10.0, 10.0, 0.0};

if (ocp.solveOcp(init, target) == 0) {
    ocp.extractSolution();
    ocp.saveTrajectoriesToJson("solution.json");
}
```

### Model Predictive Control

```cpp
#include "robot/ocp/Optimizer.hpp"

int main() {
    Ocp::Optimizer::MPC();  // Run MPC loop
    return 0;
}
```

## Algorithm Details

### Solver Configuration

- **NLP Solver**: IPOPT (Interior Point OPTimizer)
- **Max iterations**: 1000
- **Print level**: 0 (silent)

### Horizon and Timestep Selection

**Single Optimization:**
- Intervals (N): 50
- Timestep (dt): 0.2s
- Total horizon: 10s

**MPC:**
- Intervals (N): 20
- Timestep (dt): 0.2s
- Receding horizon: 4s

### Convergence

The solver typically converges in:
- 20-100 iterations for single optimization
- 10-50 iterations per MPC step

Convergence depends on:
- Initial guess quality
- Problem difficulty (obstacle density)
- State-to-goal distance

## Output Files

### ocp_solution.json

Generated by `Optimizer::Optimize()` or `Ocp::saveTrajectoriesToJson()`.

Contains complete optimized trajectory.

### robot_sim.json

Generated by `Optimizer::MPC()`.

Contains MPC simulation trajectory (actual path taken).

## Performance

Typical solve times (Intel i7, single core):
- Single OCP (50 intervals, 5 obstacles): 0.1-0.5s
- MPC iteration (20 intervals): 0.05-0.2s

## Troubleshooting

**Solver fails to converge:**
- Reduce number of obstacles
- Increase safety margin
- Better initial guess
- Increase max iterations

**Infeasible problem:**
- Check if path to goal exists
- Obstacles may block all paths
- Initial/target states may be inside obstacles
- Safety margin may be too large

**Slow performance:**
- Reduce number of intervals
- Reduce number of obstacles
- Use generated C code (faster)
- Enable compiler optimizations

## Future Enhancements

- Warm-starting with previous solution
- Adaptive horizon length
- Time-optimal formulation
- Multiple robot coordination
- Dynamic obstacles
- Soft constraints for flexibility

## See Also

- [Dynamics README](../dynamics/README.md) - Robot model used in OCP
- [Environment README](../environment/README.md) - Configuration files
- [Main C++ README](../README.md) - Overall implementation
