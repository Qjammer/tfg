cmake_minimum_required(VERSION 2.6)
project(tfg)

file(GLOB_RECURSE SOURCES_IPC "ipc/*.cpp")
file(GLOB_RECURSE SOURCES_MOD "modules/*.cpp")

add_executable(main main.cpp ${SOURCES_IPC} ${SOURCES_MOD})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -Wall")

include_directories(include)

