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
    Ocp(int N, robot::Model&& bot, double safetyMargin = 0.0, double simStep = 0.1);
    ~Ocp() = default; /**< Default destructor. */

    // methods
    int getNumIntervals() const { return m_numIntervals; } /**< Get number of intervals. */
    const robot::Model& getModel() const { return m_model; } /**< Get robot model. */
    int getNumStates() const { return m_numStates; } /**< Get number of states of dynamic model */
    int getNumControls() const { return m_numControls; } /**< Get number of controls. */
    double getSimStep() const { return m_simStep; } /**< Get simulation step size. */
    const casadi::Function& getNLPSolver() const { return m_nlpSolver; } /**< Get NLP solver function. */
    
    // Trajectory getters
    const std::vector<double>& getXTrajectory() const { return m_xTraj; }
    const std::vector<double>& getYTrajectory() const { return m_yTraj; }
    const std::vector<double>& getThetaTrajectory() const { return m_thetaTraj; }
    const std::vector<double>& getVTrajectory() const { return m_vTraj; }
    const std::vector<double>& getOmegaTrajectory() const { return m_omegaTraj; }


    // attributes

private:
    // methods
    void setupOcp(std::vector<std::vector<double>>&& obstacles={}); /**< Setup the OCP problem. */
    void generateCode(); /**< Generate code for the OCP solver. */
    void createInitialGuess(); /**< Create an initial guess for the solver. */
    int solveOcp(
        const std::vector<double>& initState={0.0, 0.0, 0.0}, 
        const std::vector<double>& targetState={10.0, 10.0, 0.0}); /**< Solve the OCP problem. */
    void extractSolution(); /**< Extract the solution from the solver. */
    void plotSolution(); /**< Plot the solution trajectories. */
    void saveTrajectoriesToJson(const std::string& filename) const; /**< Save trajectories to a JSON file. */

    // attributes
    double m_obstacleSafetyMargin; /**< Safety margin around obstacles. */
    double m_simStep;  /**< Simulation step size. */
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
    std::vector<double> m_optx; /**< Optimal decision variables. */
    std::vector<double> m_xTraj; /**< X trajectory. */
    std::vector<double> m_yTraj; /**< Y trajectory. */
    std::vector<double> m_thetaTraj; /**< Theta trajectory. */
    std::vector<double> m_vTraj; /**< Velocity trajectory. */
    std::vector<double> m_omegaTraj; /**< Angular velocity trajectory. */    

}; // Ocp

} // namespace Ocp

#endif // OCP_HPP