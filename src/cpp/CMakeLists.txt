add_library(utilities
    utilities/utilities.cpp
)

# Link utilities to nlohmann_json (header-only, just adds include paths)
target_link_libraries(utilities
    PUBLIC
        # header only library
        nlohmann_json::nlohmann_json # Corrected to use Conan's CMake target
)

# Make utilities headers available as <utilities/utilities.hpp>
target_include_directories(utilities
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}  # Allows <utilities/utilities.hpp>
)

add_library(dynamics
    robot/dynamics/model.cpp
)

# Link dynamics to its dependencies
target_link_libraries(dynamics
    PUBLIC
        casadi
        utilities  # Now dynamics can use utilities
)

# Make dynamics headers available
target_include_directories(dynamics
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/dynamics/include
)

add_executable(control_learning main.cpp)

# Link executable against found libraries
target_link_libraries(control_learning
    PRIVATE
        fmt::fmt
        # local
        dynamics
        utilities
)