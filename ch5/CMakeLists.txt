cmake_minimum_required (VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++11")

## explicitly addressed your opencv location
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

## find opencv library
find_package( OpenCV REQUIRED)

## add header
include_directories( ${OpenCV_INCLUDE_DIRS} )

project(opencvBasic)

add_executable(imageBasicOperation imageBasicOperation.cpp)

## link library
target_link_libraries( imageBasicOperation ${OpenCV_LIBS} )
