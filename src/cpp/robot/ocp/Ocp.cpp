/**
 * @file Ocp.cpp
 * @brief Implementation of the Ocp class.
 * This source file provides the implementation of the Ocp class declared
 * in Ocp.hpp, which represents an Optimal Control Problem (OCP) using CasADi.
 */

#include "Ocp.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

namespace Ocp {

Ocp::Ocp(int N, robot::Model&& bot, double simStep)
    : m_numIntervals(N),
      m_model(std::move(bot)),
      m_numStates(m_model.getStates().size1()),
      m_numControls(m_model.getControls().size1()),
      m_simStep(simStep)
{
    // Constructor implementation (if needed)
}

void Ocp::setupOcp(
    std::vector<std::vector<double>>&& obstacles, std::vector<double>&& target) {
    
    // symbolic variables for state and control trajectories
    casadi::SX state_traj{casadi::SX::sym("states", m_numStates, m_numIntervals)};
    casadi::SX control_traj{casadi::SX::sym("controls", m_numControls, m_numIntervals-1)};
    
    casadi::SX state_0{casadi::SX::sym("init_state", m_numStates, 1)};

    // symbolic variables for solver
    casadi::SXVector decision_variables;
    casadi::SXVector constraints;

    // bounds on decision variables and constraints
    std::vector<double> lb_decision_vars;
    std::vector<double> ub_decision_vars;
    std::vector<double> lb_constraints;
    std::vector<double> ub_constraints;

    std::vector<double> lb_control{
        m_model.getParameters().at("min_velocity"), 
        m_model.getParameters().at("min_angular_velocity")};
        
    std::vector<double> ub_control{
        m_model.getParameters().at("max_velocity"), 
        m_model.getParameters().at("max_angular_velocity")};

    // initial state constraints
    constraints.push_back(
        state_traj(casadi::Slice(), 0) - state_0
    );
    lb_constraints.insert(lb_constraints.end(), m_numStates, 0.0);
    ub_constraints.insert(ub_constraints.end(), m_numStates, 0.0);

    // initialize cost function
    casadi::SX cost = casadi::SX::zeros(1);

    // construct target from input vector
    casadi::SX target_state = casadi::SX::vertcat({
        casadi::SX(target[0]), 
        casadi::SX(target[1]), 
        casadi::SX(target[2])
    });

    for (int node = 0; node < m_numIntervals - 1; ++node) {
        // states
        decision_variables.push_back(state_traj(casadi::Slice(), node));
        lb_decision_vars.insert(lb_decision_vars.end(), m_numStates, -casadi::inf);
        ub_decision_vars.insert(ub_decision_vars.end(), m_numStates, casadi::inf);
        // controls
        decision_variables.push_back(control_traj(casadi::Slice(), node));
        // Create lower and upper bounds for controls at this node
        lb_decision_vars.insert(lb_decision_vars.end(), lb_control.begin(), lb_control.end());
        ub_decision_vars.insert(ub_decision_vars.end(), ub_control.begin(), ub_control.end());

        // constraints

        /* Continuity Constraints *******************/
        constraints.push_back(
            m_model.getDiscretizedDynamics()(
                casadi::SXVector{ 
                    state_traj(casadi::Slice(), node),
                    control_traj(casadi::Slice(), node),
                    casadi::DM(m_simStep)}
            )[0] - state_traj(casadi::Slice(), node + 1)
        );
        lb_constraints.insert(lb_constraints.end(), m_numStates, 0.0);
        ub_constraints.insert(ub_constraints.end(), m_numStates, 0.0);
        /*********************************************/

        /* Path Constraints **************************/
        auto x = state_traj(0, node);
        auto y = state_traj(1, node);
        for (const auto& obs : obstacles) {
            constraints.push_back(
                - casadi::SX::sq(x - obs[0]) - casadi::SX::sq(y - obs[1]) + casadi::SX::sq(obs[2])
            );
            lb_constraints.push_back(-casadi::inf);
            ub_constraints.push_back(0.0);
        }
        /*********************************************/

        /* Cost Function *****************************/
        // cost due to control effort
        cost += casadi::SX::dot(
            control_traj(casadi::Slice(), node),
            control_traj(casadi::Slice(), node)
        );

        // cost due to state deviation from [target[0], target[1], target[2]]
        cost += casadi::SX::dot(
            state_traj(casadi::Slice(), node) - target_state,
            state_traj(casadi::Slice(), node) - target_state
        );
        /*********************************************/
    }

    // Final state
    decision_variables.push_back(state_traj(casadi::Slice(), m_numIntervals - 1));
    lb_decision_vars.insert(lb_decision_vars.end(), m_numStates, -casadi::inf);
    ub_decision_vars.insert(ub_decision_vars.end(), m_numStates, casadi::inf);
    // No control at final node
    // Add cost for final state
    cost += casadi::SX::dot(
        state_traj(casadi::Slice(), m_numIntervals - 1) - target_state,
        state_traj(casadi::Slice(), m_numIntervals - 1) - target_state
    );

    // concatenate decision variables and constraints
    casadi::SX opt_variables = casadi::SX::vertcat(decision_variables);
    casadi::SX opt_constraints = casadi::SX::vertcat(constraints);

    // create NLP problem
    casadi::Dict nlp_options;
    // nlp_options["ipopt.print_level"] = 0;
    nlp_options["print_time"] = false;
    m_nlpSolver = casadi::nlpsol(
        "nlp_solver",
        "ipopt",
        casadi::SXDict{
            {"x", opt_variables},
            {"f", cost},
            {"p", casadi::SX::vertcat({state_0})},
            {"g", opt_constraints}
        },
        nlp_options
    ); 

    m_lbx = std::move(lb_decision_vars);
    m_ubx = std::move(ub_decision_vars);
    m_lbg = std::move(lb_constraints);
    m_ubg = std::move(ub_constraints);

} // setupOcp

void Ocp::createInitialGuess() {
    // create a simple initial guess (ones)
    int total_decision_vars = m_lbx.size();
    m_initialGuess.resize(total_decision_vars, 1.0);
} // createInitialGuess

void Ocp::solveOcp(std::vector<double>&& initState) {

    casadi::DMDict arg = {
        {"x0", m_initialGuess},
        {"p", initState},
        {"lbx", m_lbx},
        {"ubx", m_ubx},
        {"lbg", m_lbg},
        {"ubg", m_ubg}
    };
    
    m_sol = m_nlpSolver(arg);

    m_opt_x = m_sol.at("x").nonzeros();

    if(static_cast<int>(m_nlpSolver.stats()["success"]) == 1) {
        std::cout << "OCP solved successfully." << std::endl;
    } else {
        std::cerr << "OCP solver failed." << std::endl;
    }

} // solveOcp

void Ocp::extractSolution() {
    m_x_traj.clear();
    m_y_traj.clear();
    m_theta_traj.clear();
    m_v_traj.clear();
    m_omega_traj.clear();
    for (size_t idx{0UL}; idx < m_opt_x.size(); idx+=(m_numStates+m_numControls)) {
        m_x_traj.push_back(m_opt_x[idx]);
        m_y_traj.push_back(m_opt_x[idx+1]);
        m_theta_traj.push_back(m_opt_x[idx+2]);
        m_v_traj.push_back(m_opt_x[idx+3]);
        m_omega_traj.push_back(m_opt_x[idx+4]);
    }
    // terminal state
    size_t terminal_idx = m_opt_x.size() - (m_numStates);
    m_x_traj.push_back(m_opt_x[terminal_idx]);
    m_y_traj.push_back(m_opt_x[terminal_idx+1]);
    m_theta_traj.push_back(m_opt_x[terminal_idx+2]);
    
} // extractSolution

void Ocp::plotSolution() {
    // Plot the solution trajectories
    std::cout << "X trajectory: " << m_x_traj << std::endl;
    std::cout << "Y trajectory: " << m_y_traj << std::endl;
    std::cout << "Theta trajectory: " << m_theta_traj << std::endl;
    std::cout << "V trajectory: " << m_v_traj << std::endl;
    std::cout << "Omega trajectory: " << m_omega_traj << std::endl;

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
        {"x", m_x_traj},
        {"y", m_y_traj},
        {"theta", m_theta_traj}
    };
    
    j["controls"] = {
        {"v", m_v_traj},
        {"omega", m_omega_traj}
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