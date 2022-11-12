cmake_minimum_required(VERSION 3.22)
project(cc_reeds_shapp)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_executable(cc_reeds_shapp
        src/TurnParams.cpp
        include/TurnParams.h)
