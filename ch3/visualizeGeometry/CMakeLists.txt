cmake_minimum_required(VERSION 2.8)
project(visualizeGeometry)

set(CMAKE_CXX_STANDARD 11)

# Eigen
include_directories("/home/lab605/eigen-3.3.9")

# Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(visualizeGeometry visualizeGeometry.cpp)

# link Pangolin
target_link_libraries(visualizeGeometry ${Pangolin_LIBRARIES})

