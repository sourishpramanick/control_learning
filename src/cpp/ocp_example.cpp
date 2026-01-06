/**
 * @file ocp_example.cpp
 * @brief Example demonstrating the use of the Ocp class
 * 
 * This example shows how to:
 * 1. Create a robot model
 * 2. Set up an optimal control problem
 * 3. Solve the OCP to drive from [0, 0, 0] to a target state
 * 4. Retrieve and display the optimal trajectory
 */

#include <iostream>
#include <iomanip>
#include "robot/dynamics/include/Model.hpp"
#include "robot/ocp/include/Ocp.hpp"

int main() {
    std::cout << "========================================\n";
    std::cout << "  Optimal Control Problem Example\n";
    std::cout << "========================================\n\n";

    try {
        // Create a robot model
        std::cout << "1. Creating robot model...\n";
        robot::Model bot{};
        std::cout << "\n";

        // Create an OCP with a 50-step horizon and 0.1s timestep
        std::cout << "2. Creating OCP solver...\n";
        robot::Ocp ocp(bot, 50, 0.1);
        std::cout << "\n";

        // Set initial state (the robot starts at origin with zero orientation)
        std::cout << "3. Setting initial state...\n";
        std::vector<double> initial_state = {0.0, 0.0, 0.0};
        ocp.setInitialState(initial_state);
        std::cout << "\n";

        // Set target state (drive to x=5.0, y=3.0, theta=0.5 rad)
        std::cout << "4. Setting target state...\n";
        std::vector<double> target_state = {5.0, 3.0, 0.5};
        ocp.setTarget(target_state);
        std::cout << "\n";

        // Solve the optimal control problem
        std::cout << "5. Solving OCP...\n";
        std::cout << "   (This may take a few moments)\n\n";
        bool success = ocp.solve();

        if (success) {
            std::cout << "\n========================================\n";
            std::cout << "  Solution Found!\n";
            std::cout << "========================================\n\n";

            // Get the optimal trajectory
            auto optimal_states = ocp.getOptimalStates();
            auto optimal_controls = ocp.getOptimalControls();

            std::cout << "6. Displaying trajectory (first 5 and last 5 timesteps):\n\n";
            
            const int horizon = 50;
            const double dt = 0.1;
            const int display_count = 5;
            
            // Display first few timesteps
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "Time | x      | y      | theta  | v      | omega\n";
            std::cout << "-----+--------+--------+--------+--------+-------\n";
            
            for (int k = 0; k < std::min(display_count, horizon); ++k) {
                double t = k * dt;
                std::cout << std::setw(4) << t << " | "
                          << std::setw(6) << optimal_states(0, k).get_elements()[0] << " | "
                          << std::setw(6) << optimal_states(1, k).get_elements()[0] << " | "
                          << std::setw(6) << optimal_states(2, k).get_elements()[0] << " | "
                          << std::setw(6) << optimal_controls(0, k).get_elements()[0] << " | "
                          << std::setw(6) << optimal_controls(1, k).get_elements()[0] << "\n";
            }
            
            std::cout << " ... | ...    | ...    | ...    | ...    | ...\n";
            
            // Display last few timesteps
            for (int k = horizon - display_count; k < horizon; ++k) {
                double t = k * dt;
                std::cout << std::setw(4) << t << " | "
                          << std::setw(6) << optimal_states(0, k).get_elements()[0] << " | "
                          << std::setw(6) << optimal_states(1, k).get_elements()[0] << " | "
                          << std::setw(6) << optimal_states(2, k).get_elements()[0] << " | "
                          << std::setw(6) << optimal_controls(0, k).get_elements()[0] << " | "
                          << std::setw(6) << optimal_controls(1, k).get_elements()[0] << "\n";
            }
            
            // Display final state
            std::cout << "\nFinal state (at t=" << horizon * dt << "s):\n";
            std::cout << "  x     = " << optimal_states(0, horizon).get_elements()[0] << "\n";
            std::cout << "  y     = " << optimal_states(1, horizon).get_elements()[0] << "\n";
            std::cout << "  theta = " << optimal_states(2, horizon).get_elements()[0] << "\n";

        } else {
            std::cout << "\n========================================\n";
            std::cout << "  Failed to solve OCP\n";
            std::cout << "========================================\n";
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n========================================\n";
    std::cout << "  Example completed successfully!\n";
    std::cout << "========================================\n";

    return 0;
}
