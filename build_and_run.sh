#!/bin/bash

set -e

# Parse command line argument
BUILD_TYPE="${1:-release}"  # Default to release if no argument provided

# Convert to lowercase for case-insensitive matching
BUILD_TYPE=$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')

# Validate argument
if [[ "$BUILD_TYPE" != "release" && "$BUILD_TYPE" != "debug" ]]; then
    echo "Error: Invalid build type '$1'"
    echo "Usage: $0 [release|debug]"
    echo "  release - Build with optimizations (default)"
    echo "  debug   - Build with debug symbols and no optimizations"
    exit 1
fi

# Set variables based on build type
if [[ "$BUILD_TYPE" == "release" ]]; then
    CONAN_BUILD_TYPE="Release"
    CMAKE_PRESET="wsl-conan-release"
    BUILD_DIR="build/Release"
else
    CONAN_BUILD_TYPE="Debug"
    CMAKE_PRESET="wsl-conan-debug"
    BUILD_DIR="build/Debug"
fi

echo "=== Build Configuration ==="
echo "Build Type: $CONAN_BUILD_TYPE"
echo "CMake Preset: $CMAKE_PRESET"
echo "Build Directory: $BUILD_DIR"
echo ""

echo "=== Cleaning build directory ==="
rm -rf build

echo "=== Installing Release dependencies with Conan ==="
conan install . --output-folder=. --build=missing -s build_type=Release

echo "=== Installing Debug dependencies with Conan ==="
conan install . --output-folder=. --build=missing -s build_type=Debug

echo "=== Configuring CMake with Conan preset ==="
cmake --preset $CMAKE_PRESET

echo "=== Building the project ==="
cmake --build $BUILD_DIR

echo "=== Running the executable ==="
./$BUILD_DIR/control_learning

echo "=== Done! ==="