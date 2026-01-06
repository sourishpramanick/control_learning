/**
 * @file model.cpp
 * @brief Implements the Model class for representing robot dynamics.
 * 
 * This source file provides the implementation of the Model class declared
 * in model.hpp, which encapsulates the robot's dynamic model, including its
 * parameters, states, controls, and discretized dynamics using CasADi.
 */

#include "include/Model.hpp"

namespace robot {
using json = nlohmann::json;

Model::Model(const std::string& parameters_path)
    : m_states(casadi::SX::vertcat({
          casadi::SX::sym("x", 1),
          casadi::SX::sym("y", 1),
          casadi::SX::sym("theta", 1)
      })),
      m_controls(casadi::SX::vertcat({
          casadi::SX::sym("v", 1),
          casadi::SX::sym("omega", 1)
      }))
{
    // Load parameters from JSON file
    if (!std::filesystem::exists(parameters_path)) {
        throw std::runtime_error("Parameters file not found at: " + parameters_path);
    }
    auto json_params = utilities::load_json(parameters_path);
    std::cout << "Loaded parameters from " << parameters_path << ":\n" << json_params.dump(4) << std::endl;
    
    for (const auto& [key, value] : json_params.items())
    {
        for (const auto& [innerKey, innerValue] : value.items())
        {
            if (innerKey == "") {
                // add the key-value pair to the m_parameters map
                m_parameters[key] = innerValue.get<double>();
                continue;
            }
            try {
            m_parameters[innerKey] = innerValue.get<double>();
            } catch (const nlohmann::json::type_error&) {
                // m_parameters[innerKey] = innerValue.get<cppDict>();
                throw std::runtime_error("Nested parameter parsing not implemented.");
            }
        }
    }
    // utilities::print_map(m_parameters); // Use when nested parameters json is present
    std::cout << "Model parameters loaded successfully." << std::endl;
    std::cout << m_parameters << std::endl;

    computeContinuousDynamics();
    discretizeContinuousDynamics();
}

void Model::computeContinuousDynamics() {
    auto x = m_states(0);
    auto y = m_states(1);
    auto theta = m_states(2);
    auto v = m_controls(0);
    auto omega = m_controls(1);

    auto x_dot = v * casadi::SX::cos(theta);
    auto y_dot = v * casadi::SX::sin(theta);
    auto theta_dot = omega;

    continuous_dynamics = casadi::Function(
        "continuous_dynamics",
        {m_states, m_controls},
        {casadi::SX::vertcat({x_dot, y_dot, theta_dot})},
        {"x_y_theta", "v_omega"},
        {"states_dot"}
    );
} // computeContinuousDynamics

void Model::discretizeContinuousDynamics() {
    // Check if continuous_dynamics is defined
    if (continuous_dynamics.is_null()) {
        throw std::runtime_error("Continuous dynamics function is not defined.");
    }
    auto discretized_dynamics = casadi::simpleRK(continuous_dynamics, 5, 4); // 5 steps, 4th order

    auto next_states = discretized_dynamics(
        casadi::SXVector{
            m_states,
            m_controls,
            m_disc_step_size
        }
    )[0];

    m_discretized_dynamics = casadi::Function(
        "discretized_dynamics",
        {m_states, m_controls, m_disc_step_size},
        {next_states},
        {"x_y_theta", "v_omega", "disc_step_size"},
        {"next_x_y_theta"}
    );
    
} // discretizeContinuousDynamics

} // namespace robot