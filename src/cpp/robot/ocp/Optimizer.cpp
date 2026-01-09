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
    std::vector<double> init_state{
        initStateJson["x"].get<double>(),
        initStateJson["y"].get<double>(),
        initStateJson["theta"].get<double>()
    };
    std::cout << "Loaded initial state from JSON: " << init_state << std::endl;
    
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
    ocp.solveOcp(std::move(init_state), std::move(target));
    ocp.extractSolution();
    // ocp.plotSolution();
    ocp.saveTrajectoriesToJson("ocp_solution.json");

} // Optimize

} // Ocp