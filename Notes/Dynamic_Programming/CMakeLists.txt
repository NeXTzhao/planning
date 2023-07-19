cmake_minimum_required(VERSION 3.10)
project(OsqpEigen-Example)

# 设置c++版本
set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)

############################################################################
# matplotlib-cpp
include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)

# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
                           INTERFACE
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
                           $<INSTALL_INTERFACE:include>
                           )
target_compile_features(matplotlib_cpp INTERFACE
                        cxx_std_20
                        )
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
                      Python3::Python
                      Python3::Module
                      )
find_package(Python3 COMPONENTS NumPy)
if (Python3_NumPy_FOUND)
	target_link_libraries(matplotlib_cpp INTERFACE
	                      Python3::NumPy
	                      )
else ()
	target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif ()
install(
		TARGETS matplotlib_cpp
		EXPORT install_targets
)
############################################################################

add_executable(dp dp.cpp)
target_link_libraries(dp matplotlib_cpp ${OpenCV_LIBS})
