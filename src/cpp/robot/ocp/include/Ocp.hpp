/**
 * @file Ocp.hpp
 * @brief Defines the Ocp class for optimal control problems.
 *
 * This header declares the Ocp class, which implements an optimal control problem
 * solver using CasADi and IPOPT to drive a robot from an initial state to a target state.
 */

#ifndef ROBOT_OCP_OCP_HPP
#define ROBOT_OCP_OCP_HPP

#include <casadi/casadi.hpp>
#include <vector>
#include <Model.hpp>

/**
 * @namespace robot
 * @brief Contains classes and functions related to robot control and optimization.
 */
namespace robot {

/**
 * @class Ocp
 * @brief Implements an Optimal Control Problem solver.
 *
 * This class uses CasADi's Opti stack and IPOPT to solve trajectory optimization
 * problems for a robot, driving it from an initial state [0.0, 0.0, 0.0] to a target state.
 */
class Ocp {

public:
    /**
     * @brief Constructs an Ocp object with the given model and parameters.
     * @param model Reference to the robot dynamics model.
     * @param horizon Number of time steps in the planning horizon.
     * @param dt Time step duration (discretization step size).
     */
    Ocp(const Model& model, int horizon = 50, double dt = 0.1);

    /**
     * @brief Default destructor.
     */
    ~Ocp() = default;

    // Delete copy and move constructors/operators
    Ocp(const Ocp&) = delete;
    Ocp& operator=(const Ocp&) = delete;

    /**
     * @brief Sets the target state for the optimal control problem.
     * @param target Target state [x, y, theta].
     */
    void setTarget(const std::vector<double>& target);

    /**
     * @brief Sets the initial state for the optimal control problem.
     * @param initial Initial state [x, y, theta].
     */
    void setInitialState(const std::vector<double>& initial);

    /**
     * @brief Solves the optimal control problem.
     * @return True if the solver succeeded, false otherwise.
     */
    bool solve();

    /**
     * @brief Gets the optimal state trajectory.
     * @return Matrix containing the optimal state trajectory (horizon x state_dim).
     */
    casadi::DM getOptimalStates() const;

    /**
     * @brief Gets the optimal control trajectory.
     * @return Matrix containing the optimal control trajectory (horizon x control_dim).
     */
    casadi::DM getOptimalControls() const;

private:
    const Model& m_model; /**< Reference to the robot dynamics model. */
    int m_horizon; /**< Number of time steps in the planning horizon. */
    double m_dt; /**< Time step duration. */
    
    std::vector<double> m_initial_state; /**< Initial state [x, y, theta]. */
    std::vector<double> m_target_state; /**< Target state [x, y, theta]. */
    
    casadi::Opti m_opti; /**< CasADi optimization problem. */
    casadi::MX m_states; /**< Decision variables for states. */
    casadi::MX m_controls; /**< Decision variables for controls. */
    
    bool m_solved; /**< Flag indicating if the problem has been solved. */
    
    /**
     * @brief Sets up the optimization problem (constraints and objective).
     */
    void setupOptimization();
};

} // namespace robot

#endif // ROBOT_OCP_OCP_HPP
