cmake_minimum_required(VERSION 3.0)

project(exeigen)

set(CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/eigen3")

add_executable(basic_eigen basic_eigen.cpp)