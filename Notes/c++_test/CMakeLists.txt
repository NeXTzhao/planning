cmake_minimum_required(VERSION 3.21)
project(test)

set(CMAKE_CXX_STANDARD 14)

add_executable(test
        test.cpp)

add_executable(widget
        widget.cpp
        )

add_executable(test1
        test1.cpp
        )



add_executable(lam lambda.cpp)



add_executable(test2 test2.cpp)
add_executable(test3 test3.cpp)
add_executable(function function.cpp )

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


