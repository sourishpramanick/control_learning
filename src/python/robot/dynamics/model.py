"""
Model class for robot dynamics.

This module defines the Model class, which encapsulates the robot's dynamic model,
including its parameters, states, controls, and discretized dynamics using CasADi.
"""

import casadi as ca
from typing import List


class Model:
    """
    Represents the robot dynamics model.
    
    This class encapsulates the parameters and symbolic representations
    required to model and discretize the robot's dynamics using CasADi.
    
    Attributes:
        _parameters: Model parameters as a list of floats.
        _states: Symbolic representation of robot states (x, y, theta).
        _controls: Symbolic representation of control inputs (v, omega).
        _discretized_dynamics: CasADi Function for discretized dynamics.
        _disc_step_size: Symbolic discretization step size.
    """
    
    def __init__(self, parameters: List[float]):
        """
        Constructs a Model object with the given parameters.
        
        Args:
            parameters: A list of model parameters.
        """
        self._parameters = parameters
        
        # Define symbolic states: [x, y, theta]
        self._states = ca.vertcat(
            ca.SX.sym("x", 1),
            ca.SX.sym("y", 1),
            ca.SX.sym("theta", 1)
        )
        
        # Define symbolic controls: [v, omega]
        self._controls = ca.vertcat(
            ca.SX.sym("v", 1),
            ca.SX.sym("omega", 1)
        )
        
        # Define discretization step size
        self._disc_step_size = ca.SX.sym("disc_step_size", 1)
        
        # Initialize discretized dynamics
        self._discretized_dynamics = None
        
        # Discretize the dynamics
        self._discretize_dynamics()
    
    def _discretize_dynamics(self) -> None:
        """
        Discretizes the robot dynamics using CasADi's Runge-Kutta integration.
        
        This method creates a continuous dynamics function and discretizes it
        using a 4th-order Runge-Kutta scheme with 5 integration steps.
        """
        # Extract state variables
        x = self._states[0]
        y = self._states[1]
        theta = self._states[2]
        
        # Extract control variables
        v = self._controls[0]
        omega = self._controls[1]
        
        # Define continuous dynamics (state derivatives)
        x_dot = v * ca.cos(theta)
        y_dot = v * ca.sin(theta)
        theta_dot = omega
        
        # Create continuous dynamics function
        continuous_dynamics = ca.Function(
            "continuous_dynamics",
            [self._states, self._controls],
            [ca.vertcat(x_dot, y_dot, theta_dot)]
        )
        
        # Discretize using Runge-Kutta integration (5 steps, 4th order)
        discretized_dynamics = ca.simpleRK(continuous_dynamics, 5, 4)
        
        # Compute next states using the RK integrator
        # simpleRK expects: state, control, step_size
        next_states = discretized_dynamics(
            self._states, self._controls, self._disc_step_size
        )
        
        # Create the final discretized dynamics function
        self._discretized_dynamics = ca.Function(
            "discretized_dynamics",
            [self._states, self._controls, self._disc_step_size],
            [next_states]
        )
    
    @property
    def parameters(self) -> List[float]:
        """Get model parameters."""
        return self._parameters
    
    @property
    def states(self) -> ca.SX:
        """Get symbolic robot states."""
        return self._states
    
    @property
    def controls(self) -> ca.SX:
        """Get symbolic robot control inputs."""
        return self._controls
    
    @property
    def discretized_dynamics(self) -> ca.Function:
        """Get discretized dynamics function."""
        return self._discretized_dynamics
    
    def __repr__(self) -> str:
        """String representation of the Model."""
        return (f"Model(parameters={self._parameters}, "
                f"states={self._states.shape}, "
                f"controls={self._controls.shape})")



