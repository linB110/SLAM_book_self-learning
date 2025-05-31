cmake_minimum_required ( VERSION 2.8)

project (curve_fitting)

set(CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${CERES_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(curveFitting curveFitting.cpp)

target_link_libraries(curveFitting ${CERES_LIBRARIES} ${OpenCV_LIBS})
