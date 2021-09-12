# CMake最低版本要求
cmake_minimum_required(VERSION 3.5)

# 添加c++11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11" )

# 项目名称(任意)
project(AStar)

# 设置文件夹,添加头文件
include_directories(${AStart} AStar.h)

# 生成可执行文件 AStart可执行文件名称(项目名)  AStart.cpp是源文件名称,还有其他的可以在后面添加
add_executable(AStar AStar.cpp main.cpp)

