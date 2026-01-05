#include <iostream>
#include <fmt/core.h>
#include <casadi/casadi.hpp>

#include "robot/dynamics/include/model.hpp"

int main() {
robot::Model bot{};

std::cout << "Robot states: " << bot.getStates() << std::endl;
std::cout << "Robot controls: " << bot.getControls() << std::endl;
std::cout << "state_dot function: " << bot.getContinuousDynamics() << std::endl;
std::cout << "discretized dynamics function: " << bot.getDiscretizedDynamics() << std::endl;

std::vector<double> initState = {0.0, 0.0, 0.0};
std::vector<double> controls = {1.0, 0.1};

std::cout << "Initial state: " << initState << std::endl;
std::cout << "Control input: " << controls << std::endl;

double disc_step_size = 0.1;
auto next_state = bot.getDiscretizedDynamics()(casadi::DMVector{
    casadi::DM(initState),
    casadi::DM(controls),
    casadi::DM(disc_step_size)
})[0];
std::cout << "Next state after applying control: " << next_state << std::endl;

return 0;
}
