import casadi as ca
from robot.dynamics.model import Model

# Create model
params = [1.0, 2.0, 3.0]
bot = Model(params)

# Access properties
print(bot.states)
print(bot.controls)
print(bot.parameters)

# Use discretized dynamics
states_val = ca.DM([0, 0, 0])      # Initial position
controls_val = ca.DM([1.0, 0.5])   # Velocity and angular velocity
dt = 0.1

next_states = bot.discretized_dynamics(states_val, controls_val, dt)
print(f"Next states: {next_states}")