cmake_minimum_required(VERSION 3.0)

project(display_image)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(display_image display_image.cpp)
target_link_libraries(display_image ${OpenCV_LIBS})

add_executable(draw_obj draw_obj.cpp)
target_link_libraries(draw_obj ${OpenCV_LIBS})

add_executable(display_cam display_cam.cpp)
target_link_libraries(display_cam ${OpenCV_LIBS})

add_executable(ocv_gpu ocv_gpu.cpp)
target_link_libraries(ocv_gpu ${OpenCV_LIBS})
