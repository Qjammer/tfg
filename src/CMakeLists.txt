cmake_minimum_required(VERSION 2.6)
project(tfg)

file(GLOB_RECURSE SOURCES_IPC "ipc/*.cpp")
file(GLOB_RECURSE SOURCES_MOD "modules/*.cpp")

add_executable(main main.cpp ${SOURCES_IPC} ${SOURCES_MOD})

include_directories(include)

