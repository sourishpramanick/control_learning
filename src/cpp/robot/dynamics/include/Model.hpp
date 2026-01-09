/**
 * @file Model.hpp
 * @brief Defines the Model class for representing robot dynamics.
 *
 * This header declares the Model class, which encapsulates the robot's dynamic model,
 * including its parameters, states, controls, and discretized dynamics using CasADi.
 */

#ifndef ROBOT_DYNAMICS_MODEL_HPP
#define ROBOT_DYNAMICS_MODEL_HPP

#ifndef PARAMETERS_FILE_PATH
#define PARAMETERS_FILE_PATH "src/cpp/robot/data/parameters.json"
#endif

#include <vector>
#include <variant>
#include <casadi/casadi.hpp>
#include <utilities/Utilities.hpp>

/**
 * @namespace robot
 * @brief Contains classes and functions related to robot modeling and dynamics.
 */
namespace robot {

// Import type alias from utilities namespace
using utilities::cppDict;

/**
 * @class Model
 * @brief Represents the robot dynamics model.
 *
 * This class encapsulates the parameters and symbolic representations
 * required to model and discretize the robot's dynamics using CasADi.
 */
class Model {

public:
    Model(const std::string& parameters_path = PARAMETERS_FILE_PATH); /**< Constructor loads parameters from a JSON file. */
    ~Model() = default; /**< Default destructor. */

    Model(const Model&) = delete; /**< Copy constructor deleted. */
    Model& operator=(const Model&) = delete; /**< Copy assignment operator deleted. */

    Model(Model&&) = default; /**< Move constructor defaulted. */
    Model& operator=(Model&&) = default; /**< Move assignment operator defaulted. */

    const cppDict& getParameters() const { return m_parameters; } /**< Model parameters. */
    const casadi::SX& getStates() const { return m_states; } /**< Model states. */
    const casadi::SX& getControls() const { return m_controls; } /**< Model controls. */
    const casadi::Function& getContinuousDynamics() const { return m_continuousDynamics; } /**< Continuous dynamics function. */
    const casadi::Function& getDiscretizedDynamics() const { return m_discretizedDynamics; } /**< Discretized dynamics function. */
    const casadi::SX& getDiscStepSize() const { return m_discStepSize; } /**< Discretization step size. */

private:
    // Member variables
    cppDict m_parameters; /**< Model parameters (double or cppDict). */
    casadi::SX m_states; /**< Symbolic robot states. */
    casadi::SX m_controls; /**< Symbolic robot control inputs. */
    casadi::Function m_continuousDynamics; /**< Continuous dynamics function. */
    casadi::Function m_discretizedDynamics; /**< Discretized dynamics function. */
    casadi::SX m_discStepSize{casadi::SX::sym("discStepSize", 1)}; /**< Discretization step size. */
    // Member functions
    void computeContinuousDynamics();
    void discretizeContinuousDynamics(); /**< Discretizes the robot dynamics using CasADi. */
};

} // namespace robot

#endif // ROBOT_DYNAMICS_MODEL_HPP