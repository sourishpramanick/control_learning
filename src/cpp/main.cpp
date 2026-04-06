#include <iostream>
#include <string>
#include <vector>
#include <fmt/core.h>
#include <casadi/casadi.hpp>

#include "robot/ocp/Ocp.hpp"
#include "robot/ocp/Optimizer.hpp"
#include "nlp_solver_generated.h"

/**
 * @brief Entry point.
 *
 * Default (no arguments): reads initial_state.json / target.json from the
 * environment folder and saves the solution to ocp_solution.json.
 *
 * Dataset-generation mode (all three flags required):
 *   --init  <x> <y> <theta>   initial state (overrides initial_state.json)
 *   --target <x> <y> <theta>  target  state (overrides target.json)
 *   --output <path>           output JSON file (default: ocp_solution.json)
 */
int main(int argc, char* argv[]) {
    std::vector<double> initState;
    std::vector<double> target;
    std::string outputPath{"ocp_solution.json"};
    bool hasInit{false};
    bool hasTarget{false};

    for (int i = 1; i < argc; ++i) {
        const std::string arg{argv[i]};
        if (arg == "--init" && i + 3 < argc) {
            initState = {
                std::stod(argv[i + 1]),
                std::stod(argv[i + 2]),
                std::stod(argv[i + 3])
            };
            hasInit = true;
            i += 3;
        } else if (arg == "--target" && i + 3 < argc) {
            target = {
                std::stod(argv[i + 1]),
                std::stod(argv[i + 2]),
                std::stod(argv[i + 3])
            };
            hasTarget = true;
            i += 3;
        } else if (arg == "--output" && i + 1 < argc) {
            outputPath = argv[i + 1];
            ++i;
        } else if (arg == "--mpc") {
            Ocp::Optimizer::MPC();
            return 0;
        }
    }

    if (hasInit && hasTarget) {
        // Dataset-generation mode: init/target supplied via CLI
        // Propagate solver exit code so the Python generator can detect failures.
        return Ocp::Optimizer::Optimize(initState, target, outputPath);
    } else {
        // Interactive mode: load init/target from environment JSON files
        Ocp::Optimizer::Optimize();
    }

    return 0;
}
