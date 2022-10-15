cmake_minimum_required(VERSION 3.16)
project(test)

set(CMAKE_CXX_STANDARD 14)
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

add_executable(test
        test.cpp)

add_executable(widget
        widget.cpp
        pimlp_main.cpp)

add_executable(test1
        test1.cpp
        )
target_link_libraries(test1 matplotlib_cpp)

add_executable(lam lambda.cpp)


add_executable(test2 test2.cpp)
add_executable(test3 test3.cpp)
add_executable(function function.cpp)

find_package(osqp REQUIRED)
add_executable(osqp
        osqp_test.cpp)

# 多个库文件链接顺序问题
# 在链接命令中给出所依赖的库时，需要注意库之间的依赖顺序，依赖其他库的库一定要放到被依赖库的前面，这样才能真正避免undefined reference的错误，完成编译链接
target_link_libraries(osqp PRIVATE osqp::osqp)


add_executable(test5 test5.cpp)
add_executable(tree tree.cpp)

add_executable(A A.cpp main.cpp test2/Data.cpp test2/Data.h test2/test2.cpp test2/test2.h)
add_executable(test6 test6.cpp)
add_executable(kdtree kd-tree.cpp)
add_executable(angle angle.cpp)
add_executable(test7 test7.cpp)
add_executable(test8 test8.cpp)
add_executable(pimpl Pimlp.cpp pimlp_main.cpp)
add_executable(csv csv.cpp)
add_executable(fsm FSM.cpp)




