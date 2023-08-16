cmake_minimum_required(VERSION 3.10)
project(libOpenDrive VERSION 0.1.0 DESCRIPTION ".xodr library")

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 14)

find_package(Python3 3.8 COMPONENTS Interpreter Development NumPy REQUIRED)

include_directories(
        include
        Thirdparty
        visualization
)

add_library(OpenDrive SHARED
        src/Geometries/Arc.cpp
        src/Geometries/CubicSpline.cpp
        src/Geometries/Line.cpp
        src/Geometries/ParamPoly3.cpp
        src/Geometries/RoadGeometry.cpp
        src/Geometries/Spiral.cpp
        src/Geometries/Spiral/odrSpiral.cpp
        src/Junction.cpp
        src/Lane.cpp
        src/LaneSection.cpp
        src/Mesh.cpp
        src/OpenDriveMap.cpp
        src/RefLine.cpp
        src/Road.cpp
        src/RoadMark.cpp
        src/RoadObject.cpp
        src/RoutingGraph.cpp
        Thirdparty/pugixml/pugixml.cpp
)


add_executable(map main.cpp)
target_link_libraries(map
        OpenDrive
        Python3::Python
        Python3::NumPy)
