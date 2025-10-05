cmake_minimum_required(VERSION 3.0)

project(exstomp)

set(CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/eigen3")

add_library(stomp stomp.cpp)
add_library(utils utils.cpp)

add_executable(stomp_example stomp_example.cpp)
target_link_libraries(stomp_example stomp utils console_bridge)


# https://github.com/ros-industrial/stomp/tree/main