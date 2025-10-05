cmake_minimum_required(VERSION 3.0)
project(exbullet)
set(CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/eigen3")
include_directories("/usr/include/bullet")

# g++ -o hello hello.cpp -I /usr/include/bullet -lBulletDynamics -lBulletCollision -lLinearMath

add_executable(hello hello.cpp)
target_link_libraries(hello PUBLIC BulletDynamics BulletCollision LinearMath)

add_executable(bulletex bulletex.cpp)
target_link_libraries(bulletex PUBLIC BulletDynamics BulletCollision LinearMath)
