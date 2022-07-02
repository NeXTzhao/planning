cmake_minimum_required(VERSION 3.8)
project(ReferenceLine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE release)

include_directories(
        ${PROJECT_SOURCE_DIR}/Quintic_Curve/include
)

# find Eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
# find glog 0.6.0
find_package(glog 0.6.0 REQUIRED)
# find Ceres
#find_package(Ceres REQUIRED)

# 设置静态库文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 动态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 可执行文件目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

########################################################################
# matplotlib-cpp
#include(GNUInstallDirs)
#set(PACKAGE_NAME matplotlib_cpp)
#set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)
#
## Library target
#add_library(matplotlib_cpp INTERFACE)
#target_include_directories(matplotlib_cpp
#        INTERFACE
#        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
#        $<INSTALL_INTERFACE:include>
#        )
#target_compile_features(matplotlib_cpp INTERFACE
#        cxx_std_14
#        )
## TODO: Use `Development.Embed` component when requiring cmake >= 3.18
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
#target_link_libraries(matplotlib_cpp INTERFACE
#        Python3::Python
#        Python3::Module
#        )
#find_package(Python3 COMPONENTS NumPy)
#if (Python3_NumPy_FOUND)
#    target_link_libraries(matplotlib_cpp INTERFACE
#            Python3::NumPy
#            )
#else ()
#    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
#endif ()
#install(
#        TARGETS matplotlib_cpp
#        EXPORT install_targets
#)
########################################################################

# 生成quintic_curve
add_subdirectory(${PROJECT_SOURCE_DIR}/Quintic_Curve)
list(APPEND EXTRA_LIBS quintic_curve)
#add_executable(qp_spline src/qp_Spline_2d_main.cpp)
#target_link_libraries(qp_spline PUBLIC ${EXTRA_LIBS}  PUBLIC matplotlib_cpp PRIVATE osqp::osqp)

file(GLOB SRCS *cpp)

add_executable(ReferenceLine ${SRCS})
#target_link_libraries(ReferenceLine ${EXTRA_LIBS} Ceres::ceres glog::glog matplotlib_cpp )
#target_link_libraries(ReferenceLine ${EXTRA_LIBS} glog::glog matplotlib_cpp)
target_link_libraries(ReferenceLine ${EXTRA_LIBS} glog::glog)
#target_link_libraries(ReferenceLine ${CMAKE_SOURCE_DIR}/OpEn_solve/target/debug/libOpEn_solve.a)
target_link_libraries(ReferenceLine ${CMAKE_SOURCE_DIR}/OpEn_solve/target/release/libOpEn_solve.a)
target_link_libraries(ReferenceLine m)
target_link_libraries(ReferenceLine dl)
target_link_libraries(ReferenceLine pthread)