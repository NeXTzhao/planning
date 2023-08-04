cmake_minimum_required(VERSION 3.12)
project(lane_line)

# 添加编译选项
add_compile_options(-O0 -Wall -Wextra -Wpedantic)

set(CMAKE_CXX_STANDARD 14)

# sudo apt install nlohmann-json3-dev
find_package(nlohmann_json REQUIRED)

# Add include directories
include_directories(
        .
)

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


add_executable(lane_line_construction
        main.cpp
        lane_line_construction.cpp)
target_link_libraries(lane_line_construction PUBLIC matplotlib_cpp)
target_link_libraries(lane_line_construction PRIVATE nlohmann_json::nlohmann_json)
