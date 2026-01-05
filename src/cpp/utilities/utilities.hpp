
#include <nlohmann/json.hpp>

namespace utilities {

using json = nlohmann::json;

json load_json(const std::string& path);

} // namespace utilities