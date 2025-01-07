cmake_minimum_required(VERSION 3.10)
project(rrt)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include Eigen (replace `/usr/include/eigen3` with your Eigen installation path if different)
include_directories(/usr/include/eigen3)

# Include header files
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add the source files
add_executable(main main.cpp src/rrt.cpp)
