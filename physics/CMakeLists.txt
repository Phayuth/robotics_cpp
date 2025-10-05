cmake_minimum_required(VERSION 3.0)
project(exfcl)
set(CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/fcl")
include_directories("/usr/include/eigen3")

find_package(fcl REQUIRED)

add_executable(hello hello.cpp)
target_link_libraries(hello PUBLIC fcl)
