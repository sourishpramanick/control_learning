# Utilities

This directory contains utility functions used throughout the C++ implementation.

## Overview

The utilities module provides helper functions for common tasks such as JSON file loading and data structure printing.

## Files

- `Utilities.hpp` - Utility function declarations
- `Utilities.cpp` - Utility function implementations

## Functions

### loadJson

```cpp
nlohmann::json loadJson(const std::string& path);
```

Loads and parses a JSON file.

**Parameters:**
- `path`: Path to the JSON file (relative or absolute)

**Returns:**
- Parsed `nlohmann::json` object

**Throws:**
- `std::runtime_error` if file cannot be opened
- `std::runtime_error` if JSON parsing fails

**Example:**
```cpp
#include "utilities/Utilities.hpp"

utilities::json config = utilities::loadJson("config.json");
double value = config["key"].get<double>();
```

**Error Messages:**
- `"Could not open file: <path>"` - File not found or no read permission
- `"JSON parse error in file <path>: <details>"` - Malformed JSON

### printMap

```cpp
void printMap(const std::map<std::string, std::variant<double, cppDict>>& m);
```

Prints a map containing either double values or nested dictionaries.

**Parameters:**
- `m`: Map with string keys and values that are either doubles or nested maps

**Output Format:**
```
key1: 42.5
key2: { nested_key1: 1.0 nested_key2: 2.0 }
```

**Example:**
```cpp
#include "utilities/Utilities.hpp"

std::map<std::string, std::variant<double, cppDict>> params;
params["velocity"] = 2.0;
params["position"] = cppDict{{"x", 1.0}, {"y", 2.0}};

utilities::printMap(params);
```

**Output:**
```
velocity: 2.0
position: { x: 1.0 y: 2.0 }
```

## Type Aliases

```cpp
namespace utilities {
    using json = nlohmann::json;
    using cppDict = std::map<std::string, double>;
}
```

- `json`: Alias for `nlohmann::json` for convenience
- `cppDict`: Type for nested parameter dictionaries (string -> double mapping)

## Usage Examples

### Loading Configuration

```cpp
#include "utilities/Utilities.hpp"

// Load robot parameters
auto params = utilities::loadJson("src/cpp/robot/data/parameters.json");

// Extract values
double max_vel = params["parameters"]["max_velocity"].get<double>();
double wheel_base = params["parameters"]["wheel_base"].get<double>();

std::cout << "Max velocity: " << max_vel << std::endl;
```

### Loading Environment Data

```cpp
#include "utilities/Utilities.hpp"

// Load initial state
auto init_state_json = utilities::loadJson(
    "src/cpp/robot/environment/initial_state.json"
);

std::vector<double> init_state{
    init_state_json["x"].get<double>(),
    init_state_json["y"].get<double>(),
    init_state_json["theta"].get<double>()
};

std::cout << "Initial state: [" 
          << init_state[0] << ", " 
          << init_state[1] << ", " 
          << init_state[2] << "]" << std::endl;
```

### Loading Obstacles

```cpp
#include "utilities/Utilities.hpp"

auto obs_json = utilities::loadJson(
    "src/cpp/robot/environment/obstacles.json"
);

// Get safety margin
double margin = obs_json.value("safety_margin", 0.0);

// Parse obstacles
std::vector<std::vector<double>> obstacles;
for (const auto& [key, value] : obs_json.items()) {
    if (key == "safety_margin") continue;
    
    obstacles.push_back({
        value["x"].get<double>(),
        value["y"].get<double>(),
        value["r"].get<double>()
    });
}

std::cout << "Loaded " << obstacles.size() << " obstacles" << std::endl;
std::cout << "Safety margin: " << margin << "m" << std::endl;
```

### Error Handling

```cpp
#include "utilities/Utilities.hpp"
#include <iostream>

try {
    auto data = utilities::loadJson("config.json");
    // Process data...
} catch (const std::runtime_error& e) {
    std::cerr << "Error loading config: " << e.what() << std::endl;
    return 1;
}
```

## Implementation Details

### JSON Loading

The `loadJson` function:
1. Opens the file using `std::ifstream`
2. Checks if file opened successfully
3. Parses JSON using `nlohmann::json`
4. Catches parse errors and re-throws with context

**Benefits:**
- Centralized error handling
- Consistent error messages
- Automatic resource cleanup (RAII)

### Map Printing

The `printMap` function uses `std::variant` to handle heterogeneous value types:
- Checks type using `std::holds_alternative<T>`
- Extracts value using `std::get<T>`
- Formats output for readability

**Use Case:**
- Debugging parameter loading
- Verifying configuration values
- Development and testing (can be enabled when debugging nested parameters)

## Dependencies

- **nlohmann/json**: Modern C++ JSON library
  - Header-only
  - Intuitive API
  - Comprehensive error reporting

- **C++17 Standard Library**:
  - `<fstream>`: File I/O
  - `<iostream>`: Console output
  - `<map>`: Dictionary data structure
  - `<variant>`: Type-safe unions

## Design Considerations

### Why nlohmann/json?

Advantages:
- Intuitive syntax similar to Python
- Header-only (easy integration)
- Excellent error messages
- Good performance
- Wide adoption

Example comparison:
```cpp
// nlohmann/json (clean)
double x = json["position"]["x"].get<double>();

// Manual parsing (verbose)
// Would need to parse JSON manually with string operations
```

### Error Handling Philosophy

- **Fail fast**: Throw exceptions on errors
- **Clear messages**: Include file path in error messages
- **Context**: Include parsing details in exceptions

This ensures:
- Configuration errors caught early
- Easy debugging of JSON issues
- No silent failures

## Common Patterns

### Optional Values with Defaults

```cpp
auto config = utilities::loadJson("config.json");

// Get value with default if missing
double margin = config.value("safety_margin", 1.0);
```

### Nested Access

```cpp
auto config = utilities::loadJson("config.json");

// Access nested values
double max_vel = config["parameters"]["max_velocity"].get<double>();
```

### Iteration Over Objects

```cpp
auto config = utilities::loadJson("obstacles.json");

for (const auto& [name, obstacle] : config.items()) {
    if (name == "safety_margin") continue;
    
    double x = obstacle["x"].get<double>();
    double y = obstacle["y"].get<double>();
    double r = obstacle["r"].get<double>();
    
    std::cout << name << ": (" << x << ", " << y << "), r=" << r << std::endl;
}
```

### Type Checking

```cpp
auto config = utilities::loadJson("data.json");

if (config["value"].is_number()) {
    double val = config["value"].get<double>();
} else if (config["value"].is_string()) {
    std::string val = config["value"].get<std::string>();
}
```

## Future Enhancements

Potential additions:
- YAML file support
- Configuration validation schemas
- Environment variable substitution
- Include/reference support for JSON files
- Binary serialization for performance
- Logging utilities
- File path utilities
- String formatting helpers

## Testing

To test utility functions:

```cpp
// Test JSON loading
try {
    auto valid = utilities::loadJson("valid.json");
    std::cout << "✓ Valid JSON loaded" << std::endl;
} catch (...) {
    std::cerr << "✗ Failed to load valid JSON" << std::endl;
}

// Test error handling
try {
    auto invalid = utilities::loadJson("nonexistent.json");
    std::cerr << "✗ Should have thrown exception" << std::endl;
} catch (const std::runtime_error& e) {
    std::cout << "✓ Correctly caught error: " << e.what() << std::endl;
}
```

## See Also

- [Environment Configuration](../robot/environment/README.md) - JSON file formats
- [Model README](../robot/dynamics/README.md) - Using utilities for parameter loading
- [OCP README](../robot/ocp/README.md) - Using utilities for configuration
- [nlohmann/json Documentation](https://json.nlohmann.me/) - Full JSON library reference
