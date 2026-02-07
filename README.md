# Control Learning

A robotics control and optimization project implementing optimal control and trajectory planning for a differential drive robot using CasADi.

## Overview

This project provides implementations in both C++ and Python for:
- Robot dynamics modeling
- Optimal Control Problem (OCP) formulation and solving
- Model Predictive Control (MPC)
- Trajectory visualization and animation

The robot is modeled as a differential drive system with state variables `[x, y, theta]` (position and orientation) and control inputs `[v, omega]` (linear and angular velocities).

## Project Structure

```
control_learning/
├── src/
│   ├── cpp/           # C++ implementation (main)
│   ├── python/        # Python implementation
│   └── c/             # Generated C code
├── scripts/           # Visualization and utility scripts
├── build_and_run.sh   # Build and run script
├── CMakeLists.txt     # CMake build configuration
└── requirements.txt   # Python dependencies
```

## Features

### Core Functionality
- **Robot Dynamics**: Differential drive robot model with Runge-Kutta discretization
- **Optimal Control**: Trajectory optimization with obstacle avoidance constraints
- **MPC**: Model Predictive Control for real-time path following
- **Visualization**: Static plots and animated trajectory visualization with obstacles

### Key Components
- **Dynamics Model**: Implements continuous and discretized dynamics using CasADi
- **OCP Solver**: Formulates and solves optimal control problems with IPOPT
- **Environment**: Configurable obstacles, initial states, and target positions via JSON
- **Trajectory Export**: Saves solutions to JSON for visualization

## Requirements

### C++ Dependencies
- CMake 3.20+
- C++17 compiler
- CasADi (via Conan)
- fmt (via Conan)
- nlohmann_json (via Conan)

### Python Dependencies
- Python 3.6+
- CasADi
- NumPy
- Matplotlib

Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Building and Running

### C++ Application

1. Build using the provided script:
```bash
./build_and_run.sh
```

This will:
- Install dependencies via Conan
- Configure and build the project
- Run the OCP solver
- Generate `ocp_solution.json` trajectory file

2. Or build manually:
```bash
mkdir build && cd build
conan install .. --build=missing
cmake .. --preset conan-release
cmake --build .
./control_learning
```

### Python Application

Run the Python example:
```bash
python src/python/main.py
```

### Visualization

Visualize OCP solution trajectories:

```bash
# Simple visualization (static plot + animation)
python scripts/simple_visualize.py

# Or use the full visualizer with options
python scripts/visualize_trajectories.py ocp_solution.json --static --animate
```

## Configuration

Environment configuration files are in `src/cpp/robot/environment/`:

- `initial_state.json`: Starting position `[x, y, theta]`
- `target.json`: Goal position `[x, y, theta]`
- `obstacles.json`: Circular obstacles with positions and radii

Robot parameters are in `src/cpp/robot/data/parameters.json`:
- Velocity limits
- Angular velocity limits
- Discretization step size

## Documentation

Detailed documentation for each component:
- [Scripts Documentation](scripts/README.md)
- [C++ Implementation](src/cpp/README.md)
- [Python Implementation](src/python/README.md)

## License

[Add license information]

## Contributors

[Add contributor information]
