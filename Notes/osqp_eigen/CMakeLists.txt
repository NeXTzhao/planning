# Authors: Giulio Romualdi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

cmake_minimum_required(VERSION 3.8)
project(OsqpEigen-Example)

# 设置c++版本
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_STANDARD 17)

# 设置编译版本 bebug / release
SET(CMAKE_BUILD_TYPE Debug)

# 设置静态库文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# # 动态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 可执行文件目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/common/math
  ${PROJECT_SOURCE_DIR}/common/util
  ${PROJECT_SOURCE_DIR}/common/math/interpolation/include/
  ${PROJECT_SOURCE_DIR}/common/math/osqp_interface/include/
  ${PROJECT_SOURCE_DIR}/common/math/piecewise_jerk/include/
  
)

find_package(OsqpEigen)
find_package(Eigen3)

# Find OSQP library and headers
find_package(osqp REQUIRED)
# get_target_property(OSQP_INCLUDE_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
# include_directories(SYSTEM ${OSQP_INCLUDE_DIR})
# Link the OSQP shared library
# target_link_libraries(yourTarget PRIVATE osqp::osqp)

# or...

# # Link the OSQP static library
# target_link_libraries(yourTarget PRIVATE osqp::osqpstatic)


#matplotlib-cpp
include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)


# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_20
)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)

#添加子CMakeLists.txt
add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/interpolation)
list(APPEND EXTRA_LIBS interpolation)

# Examples
add_executable(lineInterpoltion src/lineInterpoltion.cpp)
target_link_libraries(lineInterpoltion PRIVATE matplotlib_cpp PUBLIC ${EXTRA_LIBS})
# set_target_properties(midpoints PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(splineInterpoltion src/splineInterpoltion.cpp)
target_link_libraries(splineInterpoltion PUBLIC matplotlib_cpp PUBLIC ${EXTRA_LIBS})
# set_target_properties(animation PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")


#MPCExample
add_executable(MPCExample src/MPCExample.cpp)
target_link_libraries(MPCExample OsqpEigen::OsqpEigen)


add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/osqp_interface)
list(APPEND EXTRA_LIBS osqp_csc)

add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/piecewise_jerk)
list(APPEND EXTRA_LIBS path_optimize)

add_executable(csc_conv src/csc_matrix_osqp.cpp)
# 多个库文件链接顺序问题
# 在链接命令中给出所依赖的库时，需要注意库之间的依赖顺序，依赖其他库的库一定要放到被依赖库的前面，这样才能真正避免undefined reference的错误，完成编译链接
target_link_libraries(csc_conv PUBLIC matplotlib_cpp PUBLIC ${EXTRA_LIBS} PUBLIC osqp::osqp )