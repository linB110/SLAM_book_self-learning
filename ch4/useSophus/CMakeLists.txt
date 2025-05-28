cmake_minimum_required (VERSION 2.8)

project (useSophus)

include_directories("/home/lab605/eigen-3.3.9")
include_directories("/home/lab605/Sophus")
add_subdirectory(/home/lab605/Sophus /home/lab605/Sophus/build)

add_executable (useSophus useSophus.cpp)

target_link_libraries(useSophus Sophus)
