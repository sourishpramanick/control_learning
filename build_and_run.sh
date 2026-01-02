#!/bin/bash

# Exit on any error
set -e

echo "=== Cleaning build directory ==="
rm -rf build

echo "=== Installing dependencies with Conan ==="
conan install . --output-folder=. --build=missing

echo "=== Configuring CMake with Conan preset ==="
cmake --preset wsl-conan-release

echo "=== Building the project ==="
cmake --build build/Release

echo "=== Running the executable ==="
./build/Release/control_learning

echo "=== Done! ==="
