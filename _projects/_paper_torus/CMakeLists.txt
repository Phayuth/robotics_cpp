cmake_minimum_required(VERSION 3.0)

project(paper_torus)

set(CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/eigen3")
include_directories("/usr/include/ompl-1.5")

# Add subdirectories for external libraries with relative paths
add_subdirectory("../../spatial_geometry" "${CMAKE_BINARY_DIR}/spatial_geometry")
add_subdirectory("../../robot" "${CMAKE_BINARY_DIR}/robot")

# so2 space
add_executable(paper_so2s paper_so2s.cpp)
target_link_libraries(paper_so2s PUBLIC ompl robot spatial_geometry yaml-cpp)

# ext goal case
add_executable(paper_r2s_extsgoalset paper_r2s_extsgoalset.cpp)
target_link_libraries(paper_r2s_extsgoalset PUBLIC ompl robot spatial_geometry yaml-cpp)

# r2 space
add_executable(paper_r2s paper_r2s.cpp)
target_link_libraries(paper_r2s PUBLIC ompl robot spatial_geometry yaml-cpp)


# PRM build
find_package(Boost REQUIRED COMPONENTS serialization)
add_executable(paper_prm_build paper_r2s_prm_build.cpp)
target_link_libraries(paper_prm_build PUBLIC ompl robot spatial_geometry yaml-cpp Boost::serialization)



add_executable(paper_r6s paper_r6s.cpp)
target_link_libraries(paper_r6s PUBLIC ompl robot spatial_geometry yaml-cpp)

add_executable(paper_r2s_constrained paper_r2s_constrained.cpp)
target_link_libraries(paper_r2s_constrained PUBLIC ompl robot spatial_geometry yaml-cpp)

add_executable(paper_r6s_constrained paper_r6s_constrained.cpp)
target_link_libraries(paper_r6s_constrained PUBLIC ompl robot spatial_geometry yaml-cpp)
