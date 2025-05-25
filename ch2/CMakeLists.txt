# declaration of least CMake version
cmake_minimum_required( VERSION 2.8 )

project( helloSLAM )

include_directories(${PROJECT_SOURCE_DIR})

# add library
#             lib name    source code
add_library ( hello libHelloSLAM.cpp )

# a executable program
#               program name   soure code
add_executable( helloSLAM helloSLAM.cpp)
add_executable( useHello useHello.cpp)
target_link_libraries( useHello hello)
