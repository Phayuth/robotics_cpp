cmake_minimum_required(VERSION 3.10)
project(NLoptExample)

set(CMAKE_CXX_STANDARD 11)

find_package(NLopt REQUIRED)

add_executable(nlopt_example nlopt_example.cpp)

target_link_libraries(nlopt_example PRIVATE NLopt::nlopt)
