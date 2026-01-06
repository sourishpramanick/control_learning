/**
 * @file utilities.hpp
 * @brief Utility functions for C++.
 * This header defines utility functions for all C++ modules.
 */
#include <nlohmann/json.hpp>

/**
* @namespace utilities
* @brief Namespace for utility functions.
*/
namespace utilities {

using json = nlohmann::json;

json load_json(const std::string& path); /**< Load a JSON file from the specified path. */

} // namespace utilities