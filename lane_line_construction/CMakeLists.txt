cmake_minimum_required(VERSION 3.12)
project(lane_line)

# 添加编译选项
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_CXX_STANDARD 14)

#find_package(nlohmann_json REQUIRED)

find_package(Python3 3.8 COMPONENTS Interpreter Development NumPy REQUIRED)

# Add include directories
include_directories(
        .
)

add_executable(lane_line_construction
        main.cpp
        lane_line_construction.cpp
        trajectory_planning.cpp
        bezier.cpp
)
target_link_libraries(lane_line_construction
        Python3::Python
        Python3::NumPy
        #[[nlohmann_json::nlohmann_json]])

#add_executable(pp
#        purepursuit.cpp
#)
#target_link_libraries(pp
#        Python3::Python
#        Python3::NumPy
#        #[[nlohmann_json::nlohmann_json]])