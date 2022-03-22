cmake_minimum_required(VERSION 3.0.0)
project(CMAKE_TEST)
#设置版本号
set (CMAKE_TEST_VERSION_MAJOR 1)
set (CMAKE_TEST_VERSION_MINOR 0)

# 在config.h.in中定义有USE_MATH宏，这里自定义是否启用
option(USE_MATH "Use myself provided math function" ON)
#由config.in.h生成config.h文件，生成文件的路径在cmake运行的文件下路径中，这里是build文件夹
configure_file(config.h.in config.h)

# 如果启用自定义外部库
if(USE_MATH)
  add_subdirectory(math)
  # 把外部链接库添加到变量EXTRA_LIBS变量中
  list(APPEND EXTRA_LIBS math) #math指子目录中生成的链接库名称
  # 添加外部链接库路径
  list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/math")
endif(USE_MATH)


# 设置CMAKE编译选项 c++14标准、o2优化、显示warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")
# 设置c++版本
# set(CMAKE_CXX_STANDARD 14)

# 设置编译版本 bebug / relese
SET(CMAKE_BUILD_TYPE Debug)
# SET(CMAKE_BUILD_TYPE "Release")


# -fPIC 作用于编译阶段，告诉编译器产生与位置无关代码(Position-Independent Code)，
# 则产生的代码中，没有绝对地址，全部使用相对地址，故而代码可以被加载器加载到内存的任意位置，
# 都可以正确的执行。这正是共享库所要求的，共享库被加载时，在内存的位置不是固定的。
# add_compile_options(-fPIC)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) #与上面被注释的作用相同，下面这种比较标准


#设置可执行文件输出路径 注意要放在靠前的位置提前设置
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# 设置静态链接库输出路径
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 设置动态链接库输出路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#设置是否生成compile_commands.json文件来记录编译过程
#默认不开启，开启后，生成的compile_commands.json包含所有编译单元所执行的指令。可查看编译的具体文件。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

# 添加math子目录，注意子文件要放在总CMakeLists.txt的上面，不然链接不到
# add_subdirectory(math)

# include_directories 设置包含的目录
# PROJECT_SOURCE_DIR 工程的根目录
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  # ${PROJECT_SOURCE_DIR}/math
)

# aux_source_directory 搜索所有的cpp文件
# 在当前src目录下搜索所有的cpp文件，并储存在变量DIR_SRCS中
aux_source_directory(./src DIR_SRCS)


# 编译可执行文件
add_executable(CMAKE_TEST ${DIR_SRCS})

#添加需要链接的外部路径
target_include_directories(CMAKE_TEST PUBLIC
              "${PROJECT_BINARY_DIR}" #会读取cmake运行目录下有config.in.h生成的config.h文件
              # ${EXTRA_INCLUDES}
) 

# 指定链接动态库或静态库 
# target_link_libraries(CMAKE_TEST MathFunctions_so) #在math中生成的链接库
# target_link_libraries(CMAKE_TEST ${PROJECT_SOURCE_DIR}/lib/libMathFunctions.so)

target_link_libraries(CMAKE_TEST PUBLIC ${EXTRA_LIBS})

# 指定工程需要用到的一些库
#find_package(PkgConfig)
#添加需要链接的库文件目录
# link_directories(
# /usr/lib64/
# ${GTK_LIBRARY_DIRS}
# )


