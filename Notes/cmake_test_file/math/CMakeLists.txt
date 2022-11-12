cmake_minimum_required(VERSION 3.1.0)
project(math VERSION 0.1.0)
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_LIB_SRCS)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)
# 生成链接库,默认生成的是静态库
add_library(${PROJECT_NAME} SHARED ${DIR_LIB_SRCS})
# add_library(MathFunctions_o STATIC ${DIR_LIB_SRCS})

# add_library(common STATIC util.cpp) # 生成静态库
# add_library(common SHARED util.cpp) # 生成动态库或共享库


# 其他的库都可以链接，但是自己不链接
# 指定了需求之后，可以安全地从顶层CMakeLists.txt中移除对变量EXTRA_INCLUDES的使用
target_include_directories(${PROJECT_NAME}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)


# Generate the testing executable file.
include(CTest)
# 启用测试
enable_testing()
# 添加测试可执行文件，即最后生成的可执行文件名称为：encrypt_and_decrypt_test，
# 依赖的实现文件是encrypt_and_decrypt_test.cpp
add_executable(${PROJECT_NAME}_test myMath_test.cpp)
# 寻找GTest和Threads库，两个库必需
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
# 包含GTest的头文件目录
include_directories(
  ${GTEST_INCLUDE_DIRS}
  # /usr/include/gtest
)
# 生成测试可执行文件encrypt_and_decrypt_test需要链接encrypt_and_decrypt、gtest、pthread库
target_link_libraries(${PROJECT_NAME}_test  ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )

# Enable CMake's testing runner to discover the test executable file.
# If CMake's version is greater than 3.10, use the "gtest_discover_tests" statement, otherwise use the "add_test" statement.
# 让CMake能够发现encrypt_and_decrypt_test文件，以便使用ctest命令时能够自动运行测试文件。
gtest_discover_tests(${PROJECT_NAME}_test)
# CMAKE3.10之前的版本用这种表达形式
# add_test(StringOperationTest ${PROJECT_NAME}_test)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)