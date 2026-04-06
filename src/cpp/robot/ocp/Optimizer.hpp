/**
 * @file Optimizer.hpp
 * @brief Header file for the Optimizer class.
 */

 #ifndef OPTIMIZER_HPP
 #define OPTIMIZER_HPP

#include <casadi/casadi.hpp>
#include <robot/ocp/Ocp.hpp>

namespace Ocp {
/**
 * @brief : Handler (Friend) class to optimize the OCP problem.
 */
class Optimizer {
public:
static void Optimize(); /**< Solve the given OCP problem. */
static void OptimizeCodeGen(); /**< Solve the OCP problem using generated code from solver. */
static void MPC(); /**< Run MPC loop using the OCP solver. */
private :
static constexpr int MAX_MPC_ITERATIONS = 200; /**< Maximum number of iterations for the MPC loop. */
}; // Optimizer
} // Ocp

#endif // OPTIMIZER_HPP