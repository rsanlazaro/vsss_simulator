cmake_minimum_required(VERSION 3.8)

project(box2d_simulator VERSION 0.0.1)

add_library(field  STATIC field/field.cpp)
add_library(server STATIC server/server.cpp)
add_library(robot  STATIC robot/robot.cpp)

add_executable(vsss_simulator simulator.cpp)

target_link_libraries(vsss_simulator box2d field server robot)