cmake_minimum_required(VERSION 3.21)
project(test)

set(CMAKE_CXX_STANDARD 14)

add_executable(test
        test.cpp )

add_executable(widget
        widget.cpp
        )

add_executable(test1
        test1.cpp
)
add_executable(lambda lambda.cpp)
add_executable(test2 test2.cpp)
add_executable(test3 test3.cpp)
add_executable(function function.cpp)