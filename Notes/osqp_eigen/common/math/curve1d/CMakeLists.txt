cmake_minimum_required(VERSION 3.8)
project(curve VERSION 0.1.0)
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_LIB_SRCS)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)
# 生成链接库,默认生成的是静态库
add_library(${PROJECT_NAME} SHARED ${DIR_LIB_SRCS})
# add_library(MathFunctions_o STATIC ${DIR_LIB_SRCS})

# 其他的库都可以链接，但是自己不链接 指定了需求之后，可以安全地从顶层CMakeLists.txt中移除对变量EXTRA_INCLUDES的使用
target_include_directories(${PROJECT_NAME}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)