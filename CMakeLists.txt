cmake_minimum_required(VERSION 3.18)
project(NBodyGPU LANGUAGES CXX CUDA)

# Set the CUDA version (optional, you can adjust based on your needs)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)

# Set default build type to Release if none is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add the source files
set(SOURCES
    src/main.cpp
    src/nbody.cu
)

# Add the executable
add_executable(nbody_simulation ${SOURCES})

# Include directories
target_include_directories(nbody_simulation PRIVATE include)

# Set compile flags for CUDA
target_compile_options(nbody_simulation PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>)

# Find CUDA (Optional: Some systems may need this if CMake cannot detect CUDA properly)
# find_package(CUDA REQUIRED)

# Add tests
enable_testing()

# Add test executable
add_executable(nbody_tests tests/test_nbody.cpp)

# Link libraries for tests (if using a testing framework like GoogleTest)
target_link_libraries(nbody_tests nbody_simulation)

# Add tests
add_test(NAME NBodyTests COMMAND nbody_tests)
