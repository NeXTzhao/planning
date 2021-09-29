# CMake最低版本要求
cmake_minimum_required(VERSION 3.5)

# 项目名称(任意)
project(lattice)

# 添加c++11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11" )

# 设置生成lib和bin文件的目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#############################
#设置依赖项
#############################
find_package(OpenCV REQUIRED)
# 依赖的头文件
include_directories(include)

# # 设置文件夹,添加头文件
# include_directories(${AStart} AStar.h)

# 生成可执行文件 AStart可执行文件名称(项目名)  AStart.cpp是源文件名称,还有其他的可以在后面添加
add_executable(frenet src/frenet_optimal_trajectory.cpp)
target_link_libraries(frenet ${OpenCV_LIBS})
