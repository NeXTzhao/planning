cmake_minimum_required(VERSION 3.5.1)
project(routing)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(opendrive_dir opendrive)
link_directories(${opendrive_dir}/lib)
include_directories(
        ${opendrive_dir}/src
        ${opendrive_dir}/src/third-party
        ${PROJECT_NAME}/
)

set(SOURCES_FILE router.cpp common.cpp)
add_library(${PROJECT_NAME} STATIC ${SOURCES_FILE})
target_link_libraries(${PROJECT_NAME}
        opendrive
        boost_filesystem
        )