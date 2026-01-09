/**
 * @file Utilities.hpp
 * @brief Utility functions for C++.
 * This header defines utility functions for all C++ modules.
 */
#include <map>
#include <variant>
#include <nlohmann/json.hpp>

/**
* @namespace utilities
* @brief Namespace for utility functions.
*/
namespace utilities {

using json = nlohmann::json;
using cppDict = std::map<std::string, double>;

json loadJson(const std::string& path); /**< Load a JSON file from the specified path. */

void printMap(const std::map<std::string, std::variant<double, cppDict>>& m); /**< Print a map with string keys and variant values. */
} // namespace utilities