cmake_minimum_required(VERSION 3.0)

project(example_usage)

set(CMAKE_CXX_STANDARD 11)

add_executable(use_findalt findaltconfig_use.cpp)
add_executable(use_shape shape_use.cpp)
add_executable(use_spatial spatial_transformation_use.cpp)

target_link_libraries(use_findalt PRIVATE spatial_geometry)
target_link_libraries(use_shape PRIVATE spatial_geometry)
target_link_libraries(use_spatial PRIVATE spatial_geometry)
