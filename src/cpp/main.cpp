#include <iostream>
#include <fmt/core.h>
#include <casadi/casadi.hpp>

int main() {

std::cout <<"Hello from C++!" <<std::endl;
fmt::print("Hello from fmt library, the answer is {}. \n", 42);

casadi::SX x = casadi::SX::sym("x");
casadi::SX f = x * x + 2 * x + 1;
std::cout << "Symbolic expression f(x) = " << f << std::endl;
return 0;
}
