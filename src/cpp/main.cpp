#include <iostream>
#include <vector>
#include <casadi/casadi.hpp>

// Include the generated C code header
extern "C" {
    #include "nlp_solver_generated.h"
}

int main() {
    std::cout << "=== Testing CasADi Generated NLP Solver ===" << std::endl;
    
    // Get solver information
    std::cout << "\nSolver Info:" << std::endl;
    std::cout << "  Number of inputs: " << nlp_solver_n_in() << std::endl;
    std::cout << "  Number of outputs: " << nlp_solver_n_out() << std::endl;
    
    // Print input names
    std::cout << "\nInput names:" << std::endl;
    for (casadi_int i = 0; i < nlp_solver_n_in(); ++i) {
        std::cout << "  [" << i << "] " << nlp_solver_name_in(i) << std::endl;
    }
    
    // Print output names
    std::cout << "\nOutput names:" << std::endl;
    for (casadi_int i = 0; i < nlp_solver_n_out(); ++i) {
        std::cout << "  [" << i << "] " << nlp_solver_name_out(i) << std::endl;
    }
    
    // Get workspace sizes
    casadi_int sz_arg, sz_res, sz_iw, sz_w;
    nlp_solver_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
    std::cout << "\nWorkspace requirements:" << std::endl;
    std::cout << "  sz_arg: " << sz_arg << std::endl;
    std::cout << "  sz_res: " << sz_res << std::endl;
    std::cout << "  sz_iw: " << sz_iw << std::endl;
    std::cout << "  sz_w: " << sz_w << std::endl;
    
    // Allocate memory
    int mem = nlp_solver_alloc_mem();
    if (mem < 0) {
        std::cerr << "Error: Failed to allocate memory" << std::endl;
        return 1;
    }
    
    // Initialize memory
    if (nlp_solver_init_mem(mem) != 0) {
        std::cerr << "Error: Failed to initialize memory" << std::endl;
        nlp_solver_free_mem(mem);
        return 1;
    }
    
    std::cout << "\nMemory allocated successfully (mem id: " << mem << ")" << std::endl;
    
    // Example: Set up problem data
    // You'll need to adapt this based on your actual NLP problem dimensions
    
    // Get input sparsities to determine sizes
    const casadi_int* x_sp = nlp_solver_sparsity_in(0);  // x (decision variables)
    const casadi_int* p_sp = nlp_solver_sparsity_in(1);  // p (parameters)
    
    casadi_int n_x = x_sp[1];  // number of decision variables
    casadi_int n_p = p_sp[1];  // number of parameters
    
    std::cout << "\nProblem dimensions:" << std::endl;
    std::cout << "  Decision variables (x): " << n_x << std::endl;
    std::cout << "  Parameters (p): " << n_p << std::endl;
    
    // Allocate input/output arrays
    std::vector<casadi_real> x0(n_x, 0.5);      // Initial guess
    std::vector<casadi_real> p(n_p, 0.0);       // Parameters
    std::vector<casadi_real> lbx(n_x, -1e20);   // Lower bounds on x
    std::vector<casadi_real> ubx(n_x, 1e20);    // Upper bounds on x
    std::vector<casadi_real> lbg, ubg;          // Constraint bounds (need actual sizes)
    
    // Get constraint dimensions
    const casadi_int* g_sp = nlp_solver_sparsity_out(1);  // g (constraints)
    casadi_int n_g = g_sp[1];
    lbg.resize(n_g, -1e20);
    ubg.resize(n_g, 1e20);
    
    std::cout << "  Constraints (g): " << n_g << std::endl;
    
    // Output arrays
    std::vector<casadi_real> x(n_x);           // Solution
    std::vector<casadi_real> f(1);             // Objective value
    std::vector<casadi_real> g(n_g);           // Constraint values
    std::vector<casadi_real> lam_x(n_x);       // Lagrange multipliers for x
    std::vector<casadi_real> lam_g(n_g);       // Lagrange multipliers for g
    std::vector<casadi_real> lam_p(n_p);       // Lagrange multipliers for p
    
    // Set up argument and result pointers
    const casadi_real* arg[nlp_solver_SZ_ARG];
    casadi_real* res[nlp_solver_SZ_RES];
    
    // Standard NLP solver inputs (check actual order with name_in)
    arg[0] = x0.data();    // x0 - initial guess
    arg[1] = p.data();     // p - parameters
    arg[2] = lbx.data();   // lbx
    arg[3] = ubx.data();   // ubx
    arg[4] = lbg.data();   // lbg
    arg[5] = ubg.data();   // ubg
    arg[6] = nullptr;      // lam_x0 (optional)
    arg[7] = nullptr;      // lam_g0 (optional)
    
    // Outputs
    res[0] = x.data();      // x - solution
    res[1] = f.data();      // f - objective value
    res[2] = g.data();      // g - constraints
    res[3] = lam_x.data();  // lam_x
    res[4] = lam_g.data();  // lam_g
    res[5] = lam_p.data();  // lam_p
    
    // Allocate workspace
    std::vector<casadi_int> iw(sz_iw);
    std::vector<casadi_real> w(sz_w);
    
    std::cout << "\n=== Solving NLP ===" << std::endl;
    
    // Call the solver
    int flag = nlp_solver(arg, res, iw.data(), w.data(), mem);
    
    if (flag == 0) {
        std::cout << "Solver succeeded!" << std::endl;
        std::cout << "Objective value: " << f[0] << std::endl;
        
        std::cout << "\nSolution (first 10 elements):" << std::endl;
        for (size_t i = 0; i < std::min(size_t(10), x.size()); ++i) {
            std::cout << "  x[" << i << "] = " << x[i] << std::endl;
        }
    } else {
        std::cerr << "Solver failed with flag: " << flag << std::endl;
    }
    
    // Free memory
    nlp_solver_free_mem(mem);
    
    std::cout << "\n=== Done ===" << std::endl;
    
    return 0;
}
