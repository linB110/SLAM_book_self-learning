cmake_minimum_required ( VERSION 2.8 )

project(ORB_experiments)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(feature_extraction feature_extraction.cpp)
target_link_libraries(feature_extraction  ${OpenCV_LIBS})

add_executable(pose_estimation pose_estimation.cpp)
target_link_libraries(pose_estimation  ${OpenCV_LIBS})

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})
