cmake_minimum_required(VERSION 3.0)
project(dubins_curves)

# 设置c++版本
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
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
        ${PROJECT_SOURCE_DIR}/include
)

# –coverage选项，可用来查看代码的覆盖测试情况，即查看代码中哪些部分已经被测试了或者没有被测试,注意不能开启优化选项
if (CMAKE_COMPILER_IS_GNUCC)
    option(ENABLE_COVERAGE "Enable coverage reporting for gcc/clang" FALSE)

    if (ENABLE_COVERAGE)
        add_compile_options(--coverage -O0)
    endif ()
endif ()

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

#add_subdirectory(3rd_party/google-test)

add_executable(dubins
        main.cpp src/dubins.cpp)

#target_include_directories(dubins
#        PUBLIC
#        include)

target_link_libraries(dubins PUBLIC m PRIVATE --coverage PUBLIC matplotlib_cpp)
#target_link_libraries(dubins PRIVATE --coverage)

#target_compile_options(dubins PRIVATE -Wall -Wextra -Wpedantic -std=gnu89)

#enable_testing()

#add_executable(unittest_dubins
#        tests/montecarlo_tests.cpp
#        tests/stableapi_tests.cpp)

#target_link_libraries(unittest_dubins
#        dubins
#        GTest::Main
#        GTest::GTest)
#target_link_libraries(unittest_dubins --coverage)
#
#add_test(unittest_dubins unittest_dubins)
