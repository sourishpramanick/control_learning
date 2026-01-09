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
}; // Optimizer
} // Ocp

#endif // OPTIMIZER_HPP