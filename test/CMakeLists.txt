cmake_minimum_required(VERSION 3.8)

project(box2d_test VERSION 0.0.1)

add_library(field STATIC field/field.cpp)
add_library(server STATIC server/server.cpp)

add_executable(test test.cpp)

target_link_libraries(test box2d.dll field server)