/**
 * @file Ocp.hpp
 * @brief Header file for the Ocp class.
 */

#ifndef OCP_HPP
#define OCP_HPP

#include <casadi/casadi.hpp>
#include <robot/dynamics/include/Model.hpp>

/**
 * @namespace Ocp
 * @brief Namespace for the Optimal Control Problem (OCP) classes and functions.
 */
namespace Ocp {

/**
 * @class Ocp
 * @brief Represents an Optimal Control Problem (OCP).
 *
 * This class encapsulates the formulation and solution of an optimal control problem
 * using CasADi for symbolic representation and numerical optimization.
 */
class Ocp {
    // Friend declarations
    friend class Optimizer;

public:
    /**
     * @brief Constructs an Ocp object with the given number of intervals and robot model.
     * @param N Number of intervals in the discretization.
     * @param bot Robot dynamics model.
     * @param simStep Simulation step size (default: 0.1).
     */
    Ocp(int N, robot::Model&& bot, double simStep = 0.1);
    ~Ocp() = default; /**< Default destructor. */

    // methods
    int getNumIntervals() const { return m_numIntervals; } /**< Get number of intervals. */
    const robot::Model& getModel() const { return m_model; } /**< Get robot model. */
    int getNumStates() const { return m_numStates; } /**< Get number of states of dynamic model */
    int getNumControls() const { return m_numControls; } /**< Get number of controls. */
    double getSimStep() const { return m_simStep; } /**< Get simulation step size. */
    const casadi::Function& getNLPSolver() const { return m_nlpSolver; } /**< Get NLP solver function. */
    
    // Trajectory getters
    const std::vector<double>& getXTrajectory() const { return m_x_traj; }
    const std::vector<double>& getYTrajectory() const { return m_y_traj; }
    const std::vector<double>& getThetaTrajectory() const { return m_theta_traj; }
    const std::vector<double>& getVTrajectory() const { return m_v_traj; }
    const std::vector<double>& getOmegaTrajectory() const { return m_omega_traj; }


    // attributes

private:
    // methods
    void setupOcp(std::vector<std::vector<double>>&& obstacles={});
    void createInitialGuess();
    void solveOcp();
    void extractSolution();
    void plotSolution();
    void saveTrajectoriesToJson(const std::string& filename) const;

    // attributes
    double m_simStep; 
    int m_numIntervals; /**< Number of intervals in the discretization. */
    robot::Model m_model; /**< Robot dynamics model. */
    int m_numStates; /**< Number of states in the robot model. */
    int m_numControls; /**< Number of controls in the robot model. */
    casadi::Function m_nlpSolver; /**< NLP solver function. */
    std::vector<double> m_lbx; /**< Lower bounds on decision variables. */
    std::vector<double> m_ubx; /**< Upper bounds on decision variables. */
    std::vector<double> m_lbg; /**< Lower bounds on constraints. */
    std::vector<double> m_ubg; /**< Upper bounds on constraints. */
    std::vector<double> m_initialGuess; /**< Initial guess for decision variables. */
    casadi::DMDict m_sol; /**< Solution dictionary. */
    std::vector<double> m_opt_x; /**< Optimal decision variables. */
    std::vector<double> m_x_traj; /**< X trajectory. */
    std::vector<double> m_y_traj; /**< Y trajectory. */
    std::vector<double> m_theta_traj; /**< Theta trajectory. */
    std::vector<double> m_v_traj; /**< Velocity trajectory. */
    std::vector<double> m_omega_traj; /**< Angular velocity trajectory. */    

}; // Ocp

} // namespace Ocp

#endif // OCP_HPP

