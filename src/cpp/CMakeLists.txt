add_library(dynamics
    robot/dynamics/model.cpp
)
add_executable(control_learning main.cpp)

# Link executable against found libraries
target_link_libraries(control_learning PRIVATE fmt::fmt casadi dynamics)
