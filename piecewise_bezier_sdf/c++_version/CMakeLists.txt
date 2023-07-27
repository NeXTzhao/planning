cmake_minimum_required(VERSION 3.12)
project(YourProjectName)

# 添加编译选项
add_compile_options(-O0 -Wall -Wextra -Wpedantic)

set(CMAKE_CXX_STANDARD 11)

# 添加Eigen库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(.)

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

add_executable(implicit_curve implicit_curve.cpp implicit_curve_test.cpp)
target_link_libraries(implicit_curve PUBLIC matplotlib_cpp)

add_executable(sdf implicit_curve.cpp main.cpp piecewise_bezier_curve.cpp piecewise_bezier_curve.h)
target_link_libraries(sdf PUBLIC matplotlib_cpp)

add_executable(bezier_curve bezier.cpp bezier.h bezier_test.cpp)
target_link_libraries(bezier_curve PUBLIC matplotlib_cpp)

add_executable(piecewise_bezier_curve piecewise_bezier_curve.cpp piecewise_bezier_curve_test.cpp)
target_link_libraries(piecewise_bezier_curve PUBLIC matplotlib_cpp)

add_executable(bezier_to_poly bezier_to_poly.cpp bezier_to_poly_test.cpp)
target_link_libraries(bezier_to_poly PUBLIC matplotlib_cpp)

add_executable(r_fun r_function.cpp r_function.h r_function_test.cpp implicit_curve.cpp bezier_to_poly.cpp)
target_link_libraries(r_fun PUBLIC matplotlib_cpp)

add_executable(main
        main.cpp
        bezier_to_poly.cpp
        implicit_curve.cpp
        piecewise_bezier_curve.cpp
        r_function.cpp
        )
target_link_libraries(main PUBLIC matplotlib_cpp)