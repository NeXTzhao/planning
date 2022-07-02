cmake_minimum_required(VERSION 3.10)
project(ReferenceLine)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# set(CMAKE_BUILD_TYPE release)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        rospy
        std_msgs
        )
include_directories(
        ${catkin_INCLUDE_DIRS}
        /usr/local/lib/
        /opt/ros/noetic/include/
)
#
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTORCC ON)

#include_directories(
#        ${CMAKE_CURRENT_BINARY_DIR}
#)


# find Eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# find glog
find_package(glog 0.6.0 REQUIRED)
# find Ceres
find_package(Ceres REQUIRED)

# 设置静态库文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 动态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 可执行文件目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

########################################################################
# matplotlib-cpp
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
        cxx_std_14
        )
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
        Python3::Python
        Python3::Module
        )
find_package(Python3 COMPONENTS NumPy)
if (Python3_NumPy_FOUND)
    target_link_libraries(matplotlib_cpp INTERFACE
            Python3::NumPy
            )
else ()
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif ()
install(
        TARGETS matplotlib_cpp
        EXPORT install_targets
)
########################################################################

file(GLOB SRCS *cpp)

add_executable(ReferenceLine ${SRCS})
target_link_libraries(ReferenceLine ${catkin_LIBRARIES} Ceres::ceres glog::glog matplotlib_cpp)

# target_link_libraries(ReferenceLine ${CMAKE_SOURCE_DIR}/lib/libOpEn_solve.a)
# target_link_libraries(ReferenceLine m)
# target_link_libraries(ReferenceLine dl)
# target_link_libraries(ReferenceLine pthread)