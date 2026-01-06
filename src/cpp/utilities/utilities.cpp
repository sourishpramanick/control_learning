/**
 * @file utilities.cpp
 * @brief Implementation of utility functions for C++.
 * This file contains the implementation of utility functions defined in utilities.hpp.
 */
#include "utilities.hpp"
#include <fstream>

namespace utilities {
using json = nlohmann::json;

json load_json(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + path);
    }
    json j;
    try {
        file >> j;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("JSON parse error in file " + path + ": " + std::string(e.what()));
    }
    return j;
} // load_json

} // namespace utilities