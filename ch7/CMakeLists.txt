cmake_minimum_required ( VERSION 2.8 )
set(CMAKE_CXX_STANDARD 17)

project(ORB_experiments)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    /usr/local/include
)

link_directories(/usr/local/lib)

add_executable(feature_extraction feature_extraction.cpp)
target_link_libraries(feature_extraction  ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d  ${OpenCV_LIBS})

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d  ${OpenCV_LIBS})
target_link_libraries(pose_estimation_3d2d
    ${OpenCV_LIBS}
    g2o_core
    g2o_stuff
    g2o_types_sba
    g2o_types_slam3d
    g2o_solver_csparse
)

