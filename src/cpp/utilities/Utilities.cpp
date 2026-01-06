/**
 * @file utilities.cpp
 * @brief Implementation of utility functions for C++.
 * This file contains the implementation of utility functions defined in utilities.hpp.
 */
#include "utilities.hpp"
#include <fstream>
#include <iostream>

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

void print_map(const std::map<std::string, std::variant<double, cppDict>>& m) {
    for (const auto& [key, value] : m) {
        std::cout << key << ": ";
        if (std::holds_alternative<double>(value)) {
            std::cout << std::get<double>(value) << std::endl;
        } else if (std::holds_alternative<cppDict>(value)) {
            const auto& dict = std::get<cppDict>(value);
            std::cout << "{ ";
            for (const auto& [k, v] : dict) {
                std::cout << k << ": " << v << " ";
            }
            std::cout << "}" << std::endl;
        }
    }
} // print_map

} // namespace utilities