/**
 * @file Optimizer.hpp
 * @brief Header file for the Optimizer class.
 */

 #ifndef OPTIMIZER_HPP
 #define OPTIMIZER_HPP

#include <string>
#include <vector>
#include <casadi/casadi.hpp>
#include <robot/ocp/Ocp.hpp>

namespace Ocp {
/**
 * @brief : Handler (Friend) class to optimize the OCP problem.
 */
class Optimizer {
public:
    /** Solve the OCP loading init/target from environment JSON files. */
    static void Optimize();

    /**
     * @brief Solve the OCP with an explicit initial state, target, and output path.
     *
     * Used by the dataset generator so that each call can specify independent
     * init/target pairs and write results to a unique output file without
     * touching the shared environment JSON files.
     *
     * @param initState   Initial robot state [x, y, theta].
     * @param target      Target robot state  [x, y, theta].
     * @param outputPath  File path for the resulting ocp_solution JSON.
     * @return 0 on solver success, 1 on solver failure.
     */
    static int Optimize(
        const std::vector<double>& initState,
        const std::vector<double>& target,
        const std::string& outputPath,
        int ocp_horizon = Optimizer::OCP_INTERVAL);

    static void OptimizeCodeGen(); /**< Solve the OCP problem using generated code from solver. */
    static void MPC(); /**< Run MPC loop using the OCP solver. */
    static int getOcpInterval() { return OCP_INTERVAL; } /**< Get the number of intervals used in the OCP discretization. */

private:
    static constexpr int MAX_MPC_ITERATIONS = 200; /**< Maximum number of iterations for the MPC loop. */
    static constexpr int OCP_INTERVAL = 100; /**< Number of intervals for the OCP discretization. */
    static constexpr int MPC_INTERVAL = 20; /**< Number of intervals for the MPC discretization. */
}; // Optimizer
} // namespace Ocp

#endif // OPTIMIZER_HPP