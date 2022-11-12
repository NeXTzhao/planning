cmake_minimum_required(VERSION 3.10)
project(OsqpEigen-Example)

# 设置c++版本
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_CXX_STANDARD 17)

# 设置编译版本 debug / release
# set(CMAKE_BUILD_TYPE Debug)

# 设置静态库文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 动态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 可执行文件目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
        ${PROJECT_SOURCE_DIR}/include/steering_functions/hc_cc_state_space
        ${PROJECT_SOURCE_DIR}/include/steering_functions/utilities
        ${PROJECT_SOURCE_DIR}/include/steering_functions/filter
        ${PROJECT_SOURCE_DIR}/include/steering_functions
)

# find Eigen library and headers
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# find glog 0.6.0 library and headers
find_package(glog 0.6.0 REQUIRED)
############################################################################
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
        cxx_std_20
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
############################################################################

# cc_rs曲线
aux_source_directory(${PROJECT_SOURCE_DIR}/src/hc_cc_state_space DIR_LIB_SRCS)
#set(SRC ${PROJECT_SOURCE_DIR}/src/hc_cc_state_space/*.cpp)
add_executable(rs src/cc_rs_main.cpp ${DIR_LIB_SRCS})
target_link_libraries(rs matplotlib_cpp glog::glog)
