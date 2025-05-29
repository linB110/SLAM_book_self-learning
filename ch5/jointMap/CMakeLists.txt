cmake_minimum_required(VERSION 2.8)

project(pointCloud)

set(CMAKE_CXX_STANDARD 11)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executable
add_executable(jointMap jointMap.cpp)

# Link libraries
target_link_libraries(jointMap ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

