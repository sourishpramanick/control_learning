/**
 * @file Optimizer.hpp
 * @brief Header file for the Optimizer class.
 */

 #ifndef OPTIMIZER_HPP
 #define OPTIMIZER_HPP

#include <casadi/casadi.hpp>
#include <robot/dynamics/include/Model.hpp>
#include <robot/ocp/Ocp.hpp>

namespace Ocp {
/**
 * @brief : tba
 */
class Optimizer {
public:
static void Optimize(Ocp&& ocp);
}; // Optimizer
} // Ocp

#endif // OPTIMIZER_HPP