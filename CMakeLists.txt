cmake_minimum_required(VERSION 3.16.0)

# Guard: only activate ESP-IDF build when IDF_PATH is set.
# Prevents VS Code's CMake extension from trying to configure
# this project with the wrong toolchain (cl.exe / MSVC).
if(DEFINED ENV{IDF_PATH})
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(Mterm32)
else()
    project(Mterm32 NONE)
endif()
