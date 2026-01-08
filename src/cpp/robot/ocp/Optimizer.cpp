/**
 * @brief : tba
 */

#include "Optimizer.hpp"
#include "utilities/Utilities.hpp"

namespace Ocp {

void Optimizer::Optimize(Ocp&& ocp) {
    // load initial state data
    utilities::json init_json = utilities::load_json("src/cpp/robot/environment/initial_state.json");
    std::vector<double> init_state{
        init_json["x"].get<double>(),
        init_json["y"].get<double>(),
        init_json["theta"].get<double>()
    };
    std::cout << "Loaded initial state from JSON: " << init_state << std::endl;
    
    // load target data
    utilities::json target_json = utilities::load_json("src/cpp/robot/environment/target.json");
    std::vector<double> target{
        target_json["x"].get<double>(),
        target_json["y"].get<double>(),
        target_json["theta"].get<double>()
    };
    std::cout << "Loaded target from JSON: " << target << std::endl;
    
    // load obstacle data as vectors of [x, y, r]
    utilities::json obstacle_json = utilities::load_json("src/cpp/robot/environment/obstacles.json");
    std::vector<std::vector<double>> obstacles{obstacle_json.size(), std::vector<double>(3, 0.0)};
    size_t idx{0UL};
    for (const auto& [key, value] : obstacle_json.items()) {
        obstacles[idx][0] = value["x"].get<double>();
        obstacles[idx][1] = value["y"].get<double>();
        obstacles[idx][2] = value["r"].get<double>();
        ++idx;
    }
    std::cout << "Loaded " << obstacles.size() << " obstacles from JSON." << std::endl;
    std::cout << obstacles << std::endl;

    ocp.setupOcp(std::move(obstacles), std::move(target));
    ocp.generateCode();
    ocp.createInitialGuess();
    ocp.solveOcp(std::move(init_state));
    ocp.extractSolution();
    // ocp.plotSolution();
    ocp.saveTrajectoriesToJson("ocp_solution.json");

} // Optimize

} // Ocp