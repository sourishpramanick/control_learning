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
    file >> j;
    return j;
} // load_json

} // namespace utilities