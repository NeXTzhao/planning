# 文件目录
# ├── bin
# ├── build
# ├── CMakeLists.txt
# ├── include
# │   └── Spline.h
# └── src
#     ├── main.cpp
#     └── Spline.cpp

# CMake最低版本要求
cmake_minimum_required(VERSION 3.5)

# 项目名称(任意)
project(spline)

# 添加c++11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11" )

# 利用CMakeLists.txt设置生成支持调试的.gdb文件
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")

# 项目名称
project (spline)

# 设置bin 和 lib 依赖项
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 设置头文件文件夹读取
include_directories (include)

# 把src的文件同意链接到一个变量名
aux_source_directory (src SRC_LIST)

# 生成可执行文件名
add_executable (Spline ${SRC_LIST})
