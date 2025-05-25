cmake_minimum_required(VERSION 2.8)

project(exercise)

set(CMAKE_CXX_STANDARD 11)

# Eigen
include_directories("/home/lab605/eigen-3.3.9")

add_executable(cordTransform cordTransform.cpp)

