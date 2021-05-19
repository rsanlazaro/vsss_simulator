cmake_minimum_required(VERSION 3.8)

project(box2d_test VERSION 0.0.1)

add_executable(test test.cpp)

target_link_libraries(test box2d.dll)