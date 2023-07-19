cmake_minimum_required(VERSION 2.8)
project(python_test)

include_directories(
  /home/next/anaconda3/envs/py3.8/include/python3.8
)
link_directories(
  /home/next/anaconda3/envs/py3.8/lib
)

add_executable(test
    src/test1.cpp)
target_link_libraries(test
    libpython3.8.so
)
