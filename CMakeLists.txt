cmake_minimum_required(VERSION 3.20)
project(control_learning VERSION 1.0.0 LANGUAGES C CXX)

# Force output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Language standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find libraries (from local + conan)
find_package(fmt CONFIG REQUIRED)
find_package(casadi REQUIRED)

add_subdirectory(src)
# add_subdirectory(src/cpp)


