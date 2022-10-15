cmake_minimum_required(VERSION 3.5)
project(pcl_read_pcd_by_reader)
 
find_package(PCL 1.2 REQUIRED)
 
# 加入Boost setting
# find_package(Boost COMPONENTS program_options REQUIRED )
# include_directories(${Boost_INCLUDE_DIRS})
# link_directories(${Boost_LIBRARY_DIRS})
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
add_executable(pcd_conv pcd_conv.cpp)
add_executable(pcd_view pcd_view.cpp)

 
target_link_libraries (pcd_conv ${PCL_LIBRARIES}) 
target_link_libraries (pcd_view ${PCL_LIBRARIES}) 
