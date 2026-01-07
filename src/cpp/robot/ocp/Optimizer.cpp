/**
 * @brief : tba
 */

#include "Optimizer.hpp"

namespace Ocp {

void Optimizer::Optimize(Ocp&& ocp) {

    ocp.setupOcp();
    ocp.createInitialGuess();
    ocp.solveOcp();
    ocp.extractSolution();
    // ocp.plotSolution();
    ocp.saveTrajectoriesToJson("ocp_solution.json");

} // Optimize

} // Ocp