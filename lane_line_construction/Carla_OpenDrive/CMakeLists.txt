cmake_minimum_required(VERSION 2.8.12)

project(routing_test)

# 设置c++版本
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_CXX_STANDARD 14)

# 设置编译版本 debug / release
set(CMAKE_BUILD_TYPE Debug)

# 设置静态库文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 动态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 可执行文件目录
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 自动将当前源和构建目录添加到包含路径
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(opendrive_dir opendrive)
set(routing_dir routing)
#set(FEM_Smoother_dir math/FEM_Smoother)

# find OSQP library and headers
#find_package(osqp REQUIRED)

# find Eigen library and headers
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
find_package(Python3 3.8 COMPONENTS Interpreter Development NumPy REQUIRED)

include_directories(
        ${opendrive_dir}/src
        ${opendrive_dir}/src/third-party
        ${routing_dir}/
        #				${FEM_Smoother_dir}/include
)
#link_directories(${opendrive_dir}/lib)
#link_directories(${FEM_Smoother_dir}/lib)

# FEM_Smoother
#add_subdirectory(${PROJECT_SOURCE_DIR}/optimizer/FEM_Smoother)
#list(APPEND EXTRA_LIBS discretized_points_smoothing)

set(SOURCES_FILE main.cpp)
#add_subdirectory(${routing_dir})
#list(APPEND EXTRA_LIBS routing)

#add_subdirectory(routing)
#add_subdirectory(opendrive)

add_executable(${PROJECT_NAME} ${SOURCES_FILE})
target_link_libraries(${PROJECT_NAME}
#        routing
#        opendrive
        #                      discretized_points_smoothing
        ${EXTRA_LIBS}
        boost_filesystem
        #                      osqp::osqp
        ${CMAKE_SOURCE_DIR}/lib/libopendrive.a
        ${CMAKE_SOURCE_DIR}/lib/librouting.a

)

target_link_libraries(${PROJECT_NAME}
        Python3::Python
        Python3::NumPy
)