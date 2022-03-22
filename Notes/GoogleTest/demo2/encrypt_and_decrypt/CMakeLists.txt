cmake_minimum_required(VERSION 3.1.0)
project(encrypt_and_decrypt VERSION 0.1.0)

# Generate the library file.
# 添加库文件，库文件名称为：${PROJECT_NAME}，也就是encrypt_and_decrypt
# 依赖的实现文件是encrypt_and_decrypt.cpp
add_library(${PROJECT_NAME} encrypt_and_decrypt.cpp)

# Generate the testing executable file.
include(CTest)
# 启用测试
enable_testing()
# 添加测试可执行文件，即最后生成的可执行文件名称为：encrypt_and_decrypt_test，
# 依赖的实现文件是encrypt_and_decrypt_test.cpp
add_executable(${PROJECT_NAME}_test encrypt_and_decrypt_test.cpp)
# 寻找GTest和Threads库，两个库必需
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
# 包含GTest的头文件目录
include_directories(
  ${GTEST_INCLUDE_DIRS}
  # /usr/include/gtest
)
# 生成测试可执行文件encrypt_and_decrypt_test需要链接encrypt_and_decrypt、gtest、pthread库
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )

# Enable CMake's testing runner to discover the test executable file.
# If CMake's version is greater than 3.10, use the "gtest_discover_tests" statement, otherwise use the "add_test" statement.
# 让CMake能够发现encrypt_and_decrypt_test文件，以便使用ctest命令时能够自动运行测试文件。
gtest_discover_tests(${PROJECT_NAME}_test)
# add_test(StringOperationTest ${PROJECT_NAME}_test)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)