aux_source_directory(. DIR_LIB_SRCS)

# 生成链接库,默认生成的是静态库
add_library(MathFunctions_so SHARED ${DIR_LIB_SRCS})
add_library(MathFunctions_o STATIC ${DIR_LIB_SRCS})

# add_library(common STATIC util.cpp) # 生成静态库
# add_library(common SHARED util.cpp) # 生成动态库或共享库

