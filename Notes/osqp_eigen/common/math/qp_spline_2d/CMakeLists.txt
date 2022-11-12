cmake_minimum_required(VERSION 3.8)
project(spline_2d VERSION 0.1.0)
aux_source_directory(${PROJECT_SOURCE_DIR}/src  DIR_LIB_SRCS)

# find_package(gflags REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(osqp REQUIRED)
# get_target_property(OSQP_INCLUDE_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  common/math/math_method
)

# 添加其他文件夹的.cc文件作为链接对象
set(MATH_METHOD                   
    ../math_method/angle.cc
    ../math_method/sin_table.cc
    ../math_method/math_utils.cc
    ../math_method/vec2d.cc
    ../math_method/curve_math.cc
)

# 生成链接库,默认生成的是静态库
add_library(${PROJECT_NAME} SHARED ${DIR_LIB_SRCS} ${MATH_METHOD})
# add_library(MathFunctions_o STATIC ${DIR_LIB_SRCS})

# 其他的库都可以链接，但是自己不链接 指定了需求之后，可以安全地从顶层CMakeLists.txt中移除对变量EXTRA_INCLUDES的使用
target_include_directories(${PROJECT_NAME}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)