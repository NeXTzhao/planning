cmake_minimum_required(VERSION 3.8)
project(discretized_points_smoothing)

# 设置静态库文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 动态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# find OSQP library and headers
find_package(osqp REQUIRED)

# find Eigen library and headers
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_LIB_SRCS)
include_directories(
        ${PROJECT_SOURCE_DIR}/include
)

# 生成链接库,默认生成的是静态库
add_library(${PROJECT_NAME} STATIC ${DIR_LIB_SRCS})
