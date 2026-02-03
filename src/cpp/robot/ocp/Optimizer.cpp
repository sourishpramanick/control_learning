/**
 * @file Optimizer.cpp
 * @brief Implementation of the Optimizer class.
 * This source file provides the implementation of the Optimizer class declared
 * in Optimizer.hpp, which handles the optimization of the Optimal Control Problem (OCP).
 */

#include "Optimizer.hpp"

namespace Ocp {

void Optimizer::Optimize() {
    // load initial state data
    utilities::json initStateJson = utilities::loadJson("src/cpp/robot/environment/initial_state.json");
    std::vector<double> initState{
        initStateJson["x"].get<double>(),
        initStateJson["y"].get<double>(),
        initStateJson["theta"].get<double>()
    };
    std::cout << "Loaded initial state from JSON: " << initState << std::endl;
    
    // load target data
    utilities::json targetJson = utilities::loadJson("src/cpp/robot/environment/target.json");
    std::vector<double> target{
        targetJson["x"].get<double>(),
        targetJson["y"].get<double>(),
        targetJson["theta"].get<double>()
    };
    std::cout << "Loaded target from JSON: " << target << std::endl;
    
    // Load obstacle data as vectors of [x, y, r]
    utilities::json obstacleJson = utilities::loadJson("src/cpp/robot/environment/obstacles.json");
    
    // Extract safety margin (default to 0.0 if not specified)
    double safetyMargin = obstacleJson.value("safety_margin", 0.0);
    
    // Build obstacle list
    std::vector<std::vector<double>> obstacles;
    obstacles.reserve(obstacleJson.size() - 1); // Reserve space (excluding safety_margin)
    
    for (const auto& [key, value] : obstacleJson.items()) {
        if (key == "safety_margin") {
            continue; // Skip non-obstacle entries
        }
        obstacles.push_back({
            value["x"].get<double>(),
            value["y"].get<double>(),
            value["r"].get<double>()
        });
    }
    
    std::cout << "Loaded " << obstacles.size() << " obstacles from JSON." << std::endl;
    std::cout << "Safety margin: " << safetyMargin << " m" << std::endl;
    std::cout << obstacles << std::endl;

    // Create model and OCP
    robot::Model bot{};
    Ocp ocp(50, std::move(bot), safetyMargin, 0.2);
    ocp.setupOcp(std::move(obstacles));
    // ocp.generateCode();
    ocp.createInitialGuess();
    ocp.solveOcp(initState, target);
    ocp.extractSolution();
    // ocp.plotSolution();
    ocp.saveTrajectoriesToJson("ocp_solution.json");

} // Optimize

void Optimizer::MPC() {
    // Load initial state data
    utilities::json initStateJson = utilities::loadJson("src/cpp/robot/environment/initial_state.json");
    std::vector<double> initState{
        initStateJson["x"].get<double>(),
        initStateJson["y"].get<double>(),
        initStateJson["theta"].get<double>()
    };
    std::cout << "Loaded initial state from JSON: " << initState << std::endl;

    // Load target data
    utilities::json targetJson = utilities::loadJson("src/cpp/robot/environment/target.json");
    std::vector<double> target{
        targetJson["x"].get<double>(),
        targetJson["y"].get<double>(),
        targetJson["theta"].get<double>()
    };
    std::cout << "Loaded target from JSON: " << target << std::endl;
        
    auto isClose = [](const std::vector<double>& a, const std::vector<double>& b, double tol = 0.04) {
        if (a.size() != b.size()) return false;
        for (size_t i = 0; i < a.size(); ++i) {
            if (std::abs(a[i] - b[i]) > tol) return false;
        }
        return true;
    };
    if (isClose(initState, target)) {
        std::cout << "Destination reached." << std::endl;
        return;
    }
    else {
        std::cout << "Starting MPC loop towards target." << std::endl;
    }

    // Load obstacle data as vectors of [x, y, r]
    utilities::json obstacleJson = utilities::loadJson("src/cpp/robot/environment/obstacles.json");
    double safetyMargin = obstacleJson.value("safety_margin", 0.0);
    std::vector<std::vector<double>> obstacles;
    obstacles.reserve(obstacleJson.size() - 1); // Reserve space (excluding safety_margin
    for (const auto& [key, value] : obstacleJson.items()) {
        if (key == "safety_margin") {
            continue; // Skip non-obstacle entries
        }
        obstacles.push_back({
            value["x"].get<double>(),
            value["y"].get<double>(),
            value["r"].get<double>()
        });
    }
    std::cout << "Loaded " << obstacles.size() << " obstacles from JSON." << std::endl;
    std::cout << "Obstacles: " << obstacles << std::endl;
    std::cout << "Safety margin: " << safetyMargin << " m" << std::endl;

    // Create model and OCP
    robot::Model bot{};
    Ocp ocp(20, std::move(bot), safetyMargin, 0.2);
    ocp.setupOcp(std::move(obstacles));
    ocp.createInitialGuess();

    // create json file with empty arrays or clear existing file
    {
        std::ofstream trajFileInit("robot_sim.json");
        if (trajFileInit.is_open()) {
            nlohmann::json trajJsonInit;
            trajJsonInit["states"]["x"] = nlohmann::json::array({initStateJson["x"].get<double>()});
            trajJsonInit["states"]["y"] = nlohmann::json::array({initStateJson["y"].get<double>()});
            trajJsonInit["states"]["theta"] = nlohmann::json::array({initStateJson["theta"].get<double>()});
            trajJsonInit["controls"]["v"] = nlohmann::json::array();
            trajJsonInit["controls"]["omega"] = nlohmann::json::array();
            trajJsonInit["metadata"]["num_controls"] = ocp.getNumControls();
            trajJsonInit["metadata"]["num_intervals"] = ocp.getNumIntervals();
            trajJsonInit["metadata"]["num_states"] = ocp.getNumStates();
            trajJsonInit["metadata"]["sim_step"] = ocp.getSimStep();
            trajFileInit << trajJsonInit.dump(4);
        }
    }

    while (!isClose(initState, target)) {
        if(ocp.solveOcp(initState, target)) {
            std::cerr << "MPC OCP solve failed, stopping MPC loop." << std::endl;
            ocp.extractSolution();
            ocp.saveTrajectoriesToJson("ocp_solution.json");
            break;
        }
        std::cout << "OCP solved successfully." << std::endl;
        ocp.extractSolution();
        // ocp.saveTrajectoriesToJson("ocp_solution.json");
        // Update initState for next iteration
        const std::vector<double>& xTraj = ocp.getXTrajectory();
        const std::vector<double>& yTraj = ocp.getYTrajectory();
        const std::vector<double>& thetaTraj = ocp.getThetaTrajectory();
        if (xTraj.size() < 2 || yTraj.size() < 2 || thetaTraj.size() < 2) {
            std::cerr << "Trajectory too short, cannot update state." << std::endl;
            break;
        }
        initState = {xTraj[1], yTraj[1], thetaTraj[1]};
        std::cout << "Updated state: " << initState << std::endl;

        // Read, update, and write the trajectory JSON file
        nlohmann::json trajJson;
        {
            std::ifstream trajFileIn("robot_sim.json");
            if (trajFileIn.is_open()) {
                trajFileIn >> trajJson;
            }
        }
        trajJson["states"]["x"].push_back(initState[0]);
        trajJson["states"]["y"].push_back(initState[1]);
        trajJson["states"]["theta"].push_back(initState[2]);
        trajJson["controls"]["v"].push_back(ocp.getVTrajectory()[0]);
        trajJson["controls"]["omega"].push_back(ocp.getOmegaTrajectory()[0]);
        {
            std::ofstream trajFileOut("robot_sim.json");
            if (trajFileOut.is_open()) {
                trajFileOut << trajJson.dump(4);
            }
        }
    }
    
    if (isClose(initState, target)) {
        std::cout << "✓ Destination reached successfully!" << std::endl;
    } else {
        std::cout << "✗ Did not reach destination." << std::endl;
    }
} // MPC

} // Ocp