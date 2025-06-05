cmake_minimum_required(VERSION 3.0)

project(example_usage)

set(CMAKE_CXX_STANDARD 11)

add_executable(use_col collision_use.cpp)
add_executable(use_planar_rr nonmobile_planar_rr_use.cpp)
add_executable(use_planar_rr_sim sim_planar_rr_use.cpp)

target_link_libraries(use_col PRIVATE robot)
target_link_libraries(use_planar_rr PRIVATE robot)
target_link_libraries(use_planar_rr_sim PRIVATE robot)
