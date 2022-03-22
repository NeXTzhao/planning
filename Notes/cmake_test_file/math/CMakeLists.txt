aux_source_directory(. DIR_LIB_SRCS)

# 生成链接库,默认生成的是静态库
add_library(math SHARED ${DIR_LIB_SRCS})
# add_library(MathFunctions_o STATIC ${DIR_LIB_SRCS})

# add_library(common STATIC util.cpp) # 生成静态库
# add_library(common SHARED util.cpp) # 生成动态库或共享库


# 其他的库都可以链接，但是自己不链接
target_include_directories(math
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)