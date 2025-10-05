cmake_minimum_required(VERSION 3.0)

project(exompl)

set(CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/eigen3")
include_directories("/usr/include/ompl-1.5/")

# g++ -std=c++11 -I /usr/include/eigen3 paper_r2s.cpp -o use_rrt_r2 -lompl

add_executable(test_r2s_rrt test_r2s_rrt.cpp)
target_link_libraries(test_r2s_rrt PUBLIC ompl)

add_executable(test_r6s_rrtstar test_r6s_rrtstar.cpp)
target_link_libraries(test_r6s_rrtstar PUBLIC ompl)

add_executable(test_so2 test_so2.cpp)
target_link_libraries(test_so2 PUBLIC ompl)

find_package(Boost REQUIRED COMPONENTS serialization)
add_executable(test_r2s_prm test_r2s_prm.cpp)
target_link_libraries(test_r2s_prm PUBLIC ompl Boost::serialization)
