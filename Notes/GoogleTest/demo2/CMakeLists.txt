cmake_minimum_required(VERSION 3.1.0)
project(StringOperation VERSION 0.1.0)

# 添加子目录encrypt_and_decrypt
add_subdirectory(encrypt_and_decrypt)
# 添加可执行文件，即最后生成的可执行文件名称为：string_operation，依赖的实现文件是main.cpp
add_executable(string_operation main.cpp)
# 生成可执行文件string_operation需要链接encrypt_and_decrypt库
target_link_libraries(string_operation encrypt_and_decrypt)
# 启用测试
include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
