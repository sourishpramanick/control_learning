/**
 * @file model.hpp
 * @brief Defines the Model class for representing robot dynamics.
 *
 * This header declares the Model class, which encapsulates the robot's dynamic model,
 * including its parameters, states, controls, and discretized dynamics using CasADi.
 */

/// @namespace robot
/// @brief Namespace containing all robot dynamics related classes and functions.

/**
 * @class Model
 * @brief Represents the robot dynamics model.
 *
 * The Model class provides an interface for storing and manipulating the robot's
 * dynamic parameters, states, controls, and discretized dynamics using CasADi symbolic expressions.
 */
/*
 * Model class for robot dynamics
 */

#ifndef ROBOT_DYNAMICS_MODEL_HPP
#define ROBOT_DYNAMICS_MODEL_HPP

#ifndef PARAMETERS_FILE_PATH
#define PARAMETERS_FILE_PATH "src/cpp/robot/data/parameters.json"
#endif

#include <vector>
#include <casadi/casadi.hpp>
#include <utilities/utilities.hpp>

/**
 * @namespace robot
 * @brief Contains classes and functions related to robot modeling and dynamics.
 */
namespace robot {

/**
 * @class Model
 * @brief Represents the robot dynamics model.
 *
 * This class encapsulates the parameters and symbolic representations
 * required to model and discretize the robot's dynamics using CasADi.
 */
class Model {

public:
    /**
     * @brief Constructs a Model object with the given parameters.
     * @param parameters_path Path to the parameters file.
     */
    Model(const std::string& parameters_path = PARAMETERS_FILE_PATH);

    /**
     * @brief Default destructor.
     */
    ~Model() = default;

    // Copy and move constructors/operators deleted
    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;

    const std::map<std::string, double>& getParameters() const { return m_parameters; } /**< Model parameters. */
    const casadi::SX& getStates() const { return m_states; } /**< Model states. */
    const casadi::SX& getControls() const { return m_controls; } /**< Model controls. */
    const casadi::Function& getContinuousDynamics() const { return continuous_dynamics; } /**< Continuous dynamics function. */
    const casadi::Function& getDiscretizedDynamics() const { return m_discretized_dynamics; } /**< Discretized dynamics function. */

private:
    // Member variables
    std::map<std::string, double> m_parameters; /**< Model parameters. */
    casadi::SX m_states; /**< Symbolic robot states. */
    casadi::SX m_controls; /**< Symbolic robot control inputs. */
    casadi::Function continuous_dynamics; /**< Continuous dynamics function. */
    casadi::Function m_discretized_dynamics; /**< Discretized dynamics function. */
    casadi::SX m_disc_step_size{casadi::SX::sym("disc_step_size", 1)}; /**< Discretization step size. */
    // Member functions
    void computeContinuousDynamics();
    void discretizeContinuousDynamics(); /**< Discretizes the robot dynamics using CasADi. */
};

} // namespace robot

#endif // ROBOT_DYNAMICS_MODEL_HPP