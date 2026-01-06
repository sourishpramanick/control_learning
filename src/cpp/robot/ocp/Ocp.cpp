/**
 * @file Ocp.cpp
 * @brief Implements the Ocp class for optimal control problems.
 * 
 * This source file provides the implementation of the Ocp class declared
 * in Ocp.hpp, which implements an optimal control problem solver using CasADi
 * and IPOPT to drive a robot from an initial state to a target state.
 */

#include "include/Ocp.hpp"
#include <iostream>

namespace robot {

Ocp::Ocp(const Model& model, int horizon, double dt)
    : m_model(model),
      m_horizon(horizon),
      m_dt(dt),
      m_initial_state({0.0, 0.0, 0.0}),
      m_target_state({0.0, 0.0, 0.0}),
      m_solved(false)
{
    // Initialize decision variables
    // States: (horizon+1) x 3 matrix (x, y, theta at each time step)
    m_states = m_opti.variable(3, m_horizon + 1);
    
    // Controls: horizon x 2 matrix (v, omega at each time step)
    m_controls = m_opti.variable(2, m_horizon);
    
    std::cout << "OCP initialized with horizon: " << m_horizon 
              << ", dt: " << m_dt << std::endl;
}

void Ocp::setTarget(const std::vector<double>& target) {
    if (target.size() != 3) {
        throw std::runtime_error("Target state must have 3 elements [x, y, theta]");
    }
    m_target_state = target;
    std::cout << "Target state set to: [" << target[0] << ", " 
              << target[1] << ", " << target[2] << "]" << std::endl;
}

void Ocp::setInitialState(const std::vector<double>& initial) {
    if (initial.size() != 3) {
        throw std::runtime_error("Initial state must have 3 elements [x, y, theta]");
    }
    m_initial_state = initial;
    std::cout << "Initial state set to: [" << initial[0] << ", " 
              << initial[1] << ", " << initial[2] << "]" << std::endl;
}

void Ocp::setupOptimization() {
    // Get model parameters
    const auto& params = m_model.getParameters();
    auto disc_dynamics = m_model.getDiscretizedDynamics();
    
    // Extract control bounds from parameters
    double max_v = params.at("max_velocity");
    double min_v = params.at("min_velocity");
    double max_omega = params.at("max_angular_velocity");
    double min_omega = params.at("min_angular_velocity");
    
    // Set up the objective function (minimize distance to target)
    casadi::MX cost = 0;
    
    // Terminal cost: penalize deviation from target at final state
    casadi::MX final_state = m_states(casadi::Slice(), m_horizon);
    casadi::MX target_mx = casadi::MX(casadi::DM(m_target_state));
    casadi::MX terminal_error = final_state - target_mx;
    cost += 100.0 * casadi::MX::dot(terminal_error, terminal_error);
    
    // Running cost: penalize control effort
    for (int k = 0; k < m_horizon; ++k) {
        casadi::MX u = m_controls(casadi::Slice(), k);
        cost += 0.1 * casadi::MX::dot(u, u);
        
        // Also penalize deviation from target along trajectory
        casadi::MX state_error = m_states(casadi::Slice(), k) - target_mx;
        cost += casadi::MX::dot(state_error, state_error);
    }
    
    m_opti.minimize(cost);
    
    // Set up dynamics constraints
    for (int k = 0; k < m_horizon; ++k) {
        casadi::MX x_k = m_states(casadi::Slice(), k);
        casadi::MX u_k = m_controls(casadi::Slice(), k);
        casadi::MX x_next = m_states(casadi::Slice(), k + 1);
        
        // Use the discretized dynamics from the model
        // Convert MX to SX-compatible format for the dynamics function
        casadi::MXVector disc_inputs = {x_k, u_k, casadi::MX(m_dt)};
        casadi::MX x_next_predicted = disc_dynamics(disc_inputs)[0];
        
        m_opti.subject_to(x_next == x_next_predicted);
    }
    
    // Set up initial state constraint
    casadi::MX initial_mx = casadi::MX(casadi::DM(m_initial_state));
    m_opti.subject_to(m_states(casadi::Slice(), 0) == initial_mx);
    
    // Set up control bounds
    for (int k = 0; k < m_horizon; ++k) {
        m_opti.subject_to(m_controls(0, k) >= min_v);
        m_opti.subject_to(m_controls(0, k) <= max_v);
        m_opti.subject_to(m_controls(1, k) >= min_omega);
        m_opti.subject_to(m_controls(1, k) <= max_omega);
    }
    
    // Set solver options
    casadi::Dict solver_opts;
    solver_opts["ipopt.print_level"] = 5;
    solver_opts["ipopt.max_iter"] = 1000;
    solver_opts["ipopt.tol"] = 1e-6;
    solver_opts["print_time"] = false;
    
    m_opti.solver("ipopt", solver_opts);
    
    std::cout << "Optimization problem setup completed." << std::endl;
}

bool Ocp::solve() {
    try {
        // Set up the optimization problem
        setupOptimization();
        
        std::cout << "Solving OCP from initial state [" 
                  << m_initial_state[0] << ", " << m_initial_state[1] << ", " 
                  << m_initial_state[2] << "] to target state [" 
                  << m_target_state[0] << ", " << m_target_state[1] << ", " 
                  << m_target_state[2] << "]..." << std::endl;
        
        // Solve the optimization problem
        auto sol = m_opti.solve();
        
        m_solved = true;
        std::cout << "OCP solved successfully!" << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error solving OCP: " << e.what() << std::endl;
        m_solved = false;
        return false;
    }
}

casadi::DM Ocp::getOptimalStates() const {
    if (!m_solved) {
        throw std::runtime_error("OCP has not been solved yet. Call solve() first.");
    }
    return m_opti.value(m_states);
}

casadi::DM Ocp::getOptimalControls() const {
    if (!m_solved) {
        throw std::runtime_error("OCP has not been solved yet. Call solve() first.");
    }
    return m_opti.value(m_controls);
}

} // namespace robot
