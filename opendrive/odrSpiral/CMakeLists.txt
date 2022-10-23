cmake_minimum_required(VERSION 3.8)
project(odrSpiral)

# find Eigen library and headers
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})


# STATIC library
add_library(odrSpiral STATIC src/curve.cpp)
target_include_directories(odrSpiral PUBLIC include)
set_property(TARGET odrSpiral PROPERTY POSITION_INDEPENDENT_CODE ON)

# 设置静态库文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 动态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 可执行文件目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

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
############################################################################


# Demo program
add_executable (Spiral src/main.cpp)

# Unix requires explicit static linking of math library "m" additionally
target_link_libraries (Spiral odrSpiral m matplotlib_cpp)

