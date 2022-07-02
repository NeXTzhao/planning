cmake_minimum_required(VERSION 3.15)
project(tinyxml2 VERSION 9.0.0)

set(CMAKE_CXX_STANDARD 17)

include_directories(include/)
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_LIB_SRCS)


add_executable(xml_parse ${DIR_LIB_SRCS})