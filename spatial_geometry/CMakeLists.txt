cmake_minimum_required(VERSION 3.0)

project(spatial_geometry)

set(CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/eigen3")

set(PROJECT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(SOURCE_FILES
    src/findaltconfig.cpp
    src/shape.cpp
    src/spatial_transformation.cpp)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_INCLUDE_DIR})

# build the example folder (add this only for example)
# add_subdirectory(example)