cmake_minimum_required(VERSION 3.1)
project(g2oCurveFitting)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    /usr/local/include
)

link_directories(/usr/local/lib)

add_executable(g2oFitting g2oFitting.cpp)

target_link_libraries(g2oFitting
    ${OpenCV_LIBS}
    g2o_core
    g2o_stuff
    g2o_solver_dense
    g2o_types_sba
)

