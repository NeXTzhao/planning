cmake_minimum_required(VERSION 3.10)
project(OsqpEigen-Example)

# 设置c++版本
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_CXX_STANDARD 20)

# 设置编译版本 debug / release
# set(CMAKE_BUILD_TYPE Debug)

# 设置静态库文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 动态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 可执行文件目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
		${PROJECT_SOURCE_DIR}/include
		${PROJECT_SOURCE_DIR}/common/math
		${PROJECT_SOURCE_DIR}/common/util
		${PROJECT_SOURCE_DIR}/common/math/message
		${PROJECT_SOURCE_DIR}/common/math/curve1d/include
		${PROJECT_SOURCE_DIR}/common/math/qp_spline_2d/include
		${PROJECT_SOURCE_DIR}/common/math/interpolation/include
		${PROJECT_SOURCE_DIR}/common/math/osqp_interface/include
		${PROJECT_SOURCE_DIR}/common/math/piecewise_jerk_path/include
		${PROJECT_SOURCE_DIR}/common/math/cartesian_frenet_conversion/include
		${PROJECT_SOURCE_DIR}/common/math/spiral_reference_line_smoother/include
		${PROJECT_SOURCE_DIR}/common/math/hermit_spiral_reference_line/include
		${PROJECT_SOURCE_DIR}/common/math/discretized_points_smoothing/include
		${PROJECT_SOURCE_DIR}/common/math/message
		${PROJECT_SOURCE_DIR}/common/math/reeds_shepp/include
)

# find OSQP library and headers
find_package(osqp REQUIRED)

# find Eigen library and headers
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# find glog 0.6.0 library and headers
find_package(glog 0.6.0 REQUIRED)

# find Ceres library and headers
find_package(Ceres REQUIRED)

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

#添加子CMakeLists.txt
# autoware 插值
add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/interpolation)
list(APPEND EXTRA_LIBS interpolation)
add_executable(lineInterpoltion src/lineInterpoltion.cpp)
target_link_libraries(lineInterpoltion PRIVATE matplotlib_cpp PUBLIC ${EXTRA_LIBS})
# set_target_properties(midpoints PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
add_executable(splineInterpoltion src/splineInterpoltion.cpp)
target_link_libraries(splineInterpoltion PUBLIC matplotlib_cpp PUBLIC ${EXTRA_LIBS})
# set_target_properties(animation PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# autoware osqp接口测试
add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/osqp_interface)
list(APPEND EXTRA_LIBS osqp_csc)
add_executable(csc_conv src/csc_matrix_osqp.cpp)
# 多个库文件链接顺序问题
# 在链接命令中给出所依赖的库时，需要注意库之间的依赖顺序，依赖其他库的库一定要放到被依赖库的前面，这样才能真正避免undefined reference的错误，完成编译链接
target_link_libraries(csc_conv PUBLIC matplotlib_cpp PUBLIC ${EXTRA_LIBS} PUBLIC osqp::osqp)

# # Apollo分段path优化
add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/piecewise_jerk_path)
list(APPEND EXTRA_LIBS path_optimize)
add_executable(pathoptim src/pathOptimize_main.cpp src/PathOptimize.cpp)
target_link_libraries(pathoptim PUBLIC ${EXTRA_LIBS} PUBLIC matplotlib_cpp PRIVATE osqp::osqp)

# Apollo分段样条优化
add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/qp_spline_2d)
list(APPEND EXTRA_LIBS spline_2d)
add_executable(qp_spline src/qp_Spline_2d_main.cpp)
target_link_libraries(qp_spline PUBLIC ${EXTRA_LIBS} PUBLIC matplotlib_cpp PRIVATE osqp::osqp)

# Apollo坐标转换 + 优化匹配点
# 方法一：寻找局部最优
add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/cartesian_frenet_conversion)
list(APPEND EXTRA_LIBS cartesian_frenet)
add_executable(carte_frenet_local_optim src/cartesianFrenet_local_optimum.cpp src/quinticPolynomial.cpp)
target_link_libraries(carte_frenet_local_optim PUBLIC ${EXTRA_LIBS} PUBLIC matplotlib_cpp)
# 方法二：构建kdtree搜索
add_executable(carte_frenet_kdtree src/cartesianFrenet_kdtree.cpp src/quinticPolynomial.cpp)
target_link_libraries(carte_frenet_kdtree PUBLIC ${EXTRA_LIBS} PUBLIC matplotlib_cpp)


# Apollo分段螺旋曲线优化参考线(Ipopt库求解)  这个跟quintic_curve冲突，需要屏蔽一个
# 添加Ipopt求解库一定要街上系统库文件的链接
 include_directories(/usr/local/include)
 link_directories(/usr/local/lib)
 add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/spiral_reference_line_smoother)
 add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/hermit_spiral_reference_line)
 list(APPEND EXTRA_LIBS spiral_smoother hermit_spiral)
 # list(APPEND EXTRA_LIBS hermit_spiral)
 add_executable(spiral_refrencesline_smoother src/spiral_smoother_main.cpp)
 target_link_libraries(spiral_refrencesline_smoother ${EXTRA_LIBS} ipopt osqp::osqp Ceres::ceres glog::glog matplotlib_cpp)

 
# 从离散圆中得到坐标
add_executable(circle src/circle.cpp)
target_link_libraries(circle PUBLIC matplotlib_cpp)


# 生成五次多项式轨迹 (不按时间采样)
# add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/curve1d)
# list(APPEND EXTRA_LIBS curve)
# add_executable(quintic_curve src/quintic_curve.cpp)
# target_link_libraries(quintic_curve PUBLIC ${EXTRA_LIBS} matplotlib_cpp)


# Apollo离散点平滑
add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/discretized_points_smoothing)
list(APPEND EXTRA_LIBS discretized_points_smoothing)
add_executable(dis_points_smooth src/discretized_points_smoothing_main.cpp)
target_link_libraries(dis_points_smooth ${EXTRA_LIBS} matplotlib_cpp osqp::osqp glog::glog)

# Apollo rs曲线
# add_subdirectory(${PROJECT_SOURCE_DIR}/common/math/reeds_shepp)
# list(APPEND EXTRA_LIBS reeds_shepp)
# add_executable(rs src/reeds_shepp_main.cpp)
# target_link_libraries(rs ${EXTRA_LIBS} matplotlib_cpp `)
