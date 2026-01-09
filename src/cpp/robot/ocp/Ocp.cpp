/**
 * @file Ocp.cpp
 * @brief Implementation of the Ocp class.
 * This source file provides the implementation of the Ocp class declared
 * in Ocp.hpp, which represents an Optimal Control Problem (OCP) using CasADi.
 */

#include "Ocp.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <unistd.h>

namespace Ocp {

Ocp::Ocp(int N, robot::Model&& bot, double safetyMargin, double simStep)
    : m_numIntervals(N),
      m_model(std::move(bot)),
      m_numStates(m_model.getStates().size1()),
      m_numControls(m_model.getControls().size1()),
      m_simStep(simStep),
      m_obstacleSafetyMargin(safetyMargin)
{
    // Constructor implementation (if needed)
}

void Ocp::setupOcp(
    std::vector<std::vector<double>>&& obstacles) {
    
    // symbolic variables for state and control trajectories
    casadi::SX stateTraj{casadi::SX::sym("states", m_numStates, m_numIntervals)};
    casadi::SX controlTraj{casadi::SX::sym("controls", m_numControls, m_numIntervals-1)};

    casadi::SX initState{casadi::SX::sym("initState", m_numStates, 1)};

    // symbolic variables for solver
    casadi::SXVector decisionVariables;
    casadi::SXVector constraints;

    // bounds on decision variables and constraints
    std::vector<double> lbDecisionVars;
    std::vector<double> ubDecisionVars;
    std::vector<double> lbConstraints;
    std::vector<double> ubConstraints;

    std::vector<double> lbControl{
        m_model.getParameters().at("min_velocity"), 
        m_model.getParameters().at("min_angular_velocity")};
        
    std::vector<double> ubControl{
        m_model.getParameters().at("max_velocity"), 
        m_model.getParameters().at("max_angular_velocity")};

    // initial state constraints
    constraints.push_back(
        stateTraj(casadi::Slice(), 0) - initState
    );
    lbConstraints.insert(lbConstraints.end(), m_numStates, 0.0);
    ubConstraints.insert(ubConstraints.end(), m_numStates, 0.0);

    // initialize cost function
    casadi::SX cost = casadi::SX::zeros(1);

    // construct target from input vector
    casadi::SX targetState{casadi::SX::sym("targetState", m_numStates, 1)};

    for (int node = 0; node < m_numIntervals - 1; ++node) {
        // states
        decisionVariables.push_back(stateTraj(casadi::Slice(), node));
        lbDecisionVars.insert(lbDecisionVars.end(), m_numStates, -casadi::inf);
        ubDecisionVars.insert(ubDecisionVars.end(), m_numStates, casadi::inf);
        // controls
        decisionVariables.push_back(controlTraj(casadi::Slice(), node));
        // Create lower and upper bounds for controls at this node
        lbDecisionVars.insert(lbDecisionVars.end(), lbControl.begin(), lbControl.end());
        ubDecisionVars.insert(ubDecisionVars.end(), ubControl.begin(), ubControl.end());

        // constraints

        /* Continuity Constraints *******************/
        constraints.push_back(
            m_model.getDiscretizedDynamics()(
                casadi::SXVector{ 
                    stateTraj(casadi::Slice(), node),
                    controlTraj(casadi::Slice(), node),
                    casadi::DM(m_simStep)}
            )[0] - stateTraj(casadi::Slice(), node + 1)
        );
        lbConstraints.insert(lbConstraints.end(), m_numStates, 0.0);
        ubConstraints.insert(ubConstraints.end(), m_numStates, 0.0);
        /*********************************************/

        /* Path Constraints **************************/
        auto x = stateTraj(0, node);
        auto y = stateTraj(1, node);
        for (const auto& obs : obstacles) {
            constraints.push_back(
                - casadi::SX::sq(x - obs[0]) - casadi::SX::sq(y - obs[1]) + casadi::SX::sq(obs[2])
            );
            lbConstraints.push_back(-casadi::inf);
            ubConstraints.push_back(-m_obstacleSafetyMargin); // safety margin
        }
        /*********************************************/

        /* Cost Function *****************************/
        // cost due to control effort
        cost += casadi::SX::dot(
            controlTraj(casadi::Slice(), node),
            controlTraj(casadi::Slice(), node)
        );

        // cost due to state deviation from [target[0], target[1], target[2]]
        cost += casadi::SX::dot(
            stateTraj(casadi::Slice(), node) - targetState,
            stateTraj(casadi::Slice(), node) - targetState
        );
        /*********************************************/
    }

    // Final state
    decisionVariables.push_back(stateTraj(casadi::Slice(), m_numIntervals - 1));
    lbDecisionVars.insert(lbDecisionVars.end(), m_numStates, -casadi::inf);
    ubDecisionVars.insert(ubDecisionVars.end(), m_numStates, casadi::inf);
    // No control at final node
    // Add cost for final state
    cost += casadi::SX::dot(
        stateTraj(casadi::Slice(), m_numIntervals - 1) - targetState,
        stateTraj(casadi::Slice(), m_numIntervals - 1) - targetState
    );

    // concatenate decision variables and constraints
    casadi::SX optVariables = casadi::SX::vertcat(decisionVariables);
    casadi::SX optConstraints = casadi::SX::vertcat(constraints);

    // create NLP problem
    casadi::Dict nlp_options;
    nlp_options["ipopt.print_level"] = 0;
    nlp_options["print_time"] = true;
    m_nlpSolver = casadi::nlpsol(
        "nlp_solver",
        "ipopt",
        casadi::SXDict{
            {"x", optVariables},
            {"f", cost},
            {"p", casadi::SX::vertcat({initState, targetState})},
            {"g", optConstraints}
        },
        nlp_options
    ); 

    m_lbx = std::move(lbDecisionVars);
    m_ubx = std::move(ubDecisionVars);
    m_lbg = std::move(lbConstraints);
    m_ubg = std::move(ubConstraints);
} // setupOcp

void Ocp::generateCode() {

    // Save current working directory
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) == nullptr) {
    throw std::runtime_error("Failed to get current directory");
    }

    // Target directory and filename
    std::string gen_dir = "src/c/robot/generated_code";
    std::string gen_filename = "nlp_solver_generated.c";
    casadi::Dict gen_opts;
    gen_opts["with_header"] = true;

    // Change to target directory, generate code, then restore directory
    if (chdir(gen_dir.c_str()) == 0) {
        m_nlpSolver.generate(gen_filename, gen_opts);
        if (chdir(cwd) != 0) {
            std::cerr << "Warning: Failed to restore directory" << std::endl;
        }
    } else {
        std::cerr << "Error: Could not change directory to " << gen_dir << std::endl;
    }

} // generateCode

void Ocp::createInitialGuess() {
    // create a simple initial guess (ones)
    int totalDecisionVars = m_lbx.size();
    m_initialGuess.resize(totalDecisionVars, 1.0);
} // createInitialGuess

void Ocp::solveOcp(std::vector<double>&& initState, std::vector<double>&& targetState) {

    casadi::DMDict arg = {
        {"x0", m_initialGuess},
        {"p", casadi::DM::vertcat({casadi::DM(initState), casadi::DM(targetState)})},
        {"lbx", m_lbx},
        {"ubx", m_ubx},
        {"lbg", m_lbg},
        {"ubg", m_ubg}
    };
    
    m_sol = m_nlpSolver(arg);

    m_optx = m_sol.at("x").nonzeros();

    if(static_cast<int>(m_nlpSolver.stats()["success"]) == 1) {
        std::cout << "OCP solved successfully." << std::endl;
    } else {
        std::cerr << "OCP solver failed." << std::endl;
    }

} // solveOcp

void Ocp::extractSolution() {
    m_xTraj.clear();
    m_yTraj.clear();
    m_thetaTraj.clear();
    m_vTraj.clear();
    m_omegaTraj.clear();
    for (size_t idx{0UL}; idx < m_optx.size(); idx+=(m_numStates+m_numControls)) {
        m_xTraj.push_back(m_optx[idx]);
        m_yTraj.push_back(m_optx[idx+1]);
        m_thetaTraj.push_back(m_optx[idx+2]);
        m_vTraj.push_back(m_optx[idx+3]);
        m_omegaTraj.push_back(m_optx[idx+4]);
    }
    // terminal state
    size_t terminal_idx = m_optx.size() - (m_numStates);
    m_xTraj.push_back(m_optx[terminal_idx]);
    m_yTraj.push_back(m_optx[terminal_idx+1]);
    m_thetaTraj.push_back(m_optx[terminal_idx+2]);

} // extractSolution

void Ocp::plotSolution() {
    // Plot the solution trajectories
    std::cout << "X trajectory: " << m_xTraj << std::endl;
    std::cout << "Y trajectory: " << m_yTraj << std::endl;
    std::cout << "Theta trajectory: " << m_thetaTraj << std::endl;
    std::cout << "V trajectory: " << m_vTraj << std::endl;
    std::cout << "Omega trajectory: " << m_omegaTraj << std::endl;

} // plotSolution

void Ocp::saveTrajectoriesToJson(const std::string& filename) const {
    nlohmann::json j;
    
    j["metadata"] = {
        {"num_intervals", m_numIntervals},
        {"sim_step", m_simStep},
        {"num_states", m_numStates},
        {"num_controls", m_numControls}
    };
    
    j["states"] = {
        {"x", m_xTraj},
        {"y", m_yTraj},
        {"theta", m_thetaTraj}
    };
    
    j["controls"] = {
        {"v", m_vTraj},
        {"omega", m_omegaTraj}
    };
    
    // Save to file
    std::ofstream file(filename);
    if (file.is_open()) {
        file << j.dump(4);  // Pretty print with 4-space indentation
        file.close();
        std::cout << "Trajectories saved to: " << filename << std::endl;
    } else {
        std::cerr << "Error: Could not open file " << filename << std::endl;
    }
} // saveTrajectoriesToJson
    
} // namespace Ocp