#!/bin/bash
if [ -d build ];then
  rm -rf build
fi

# -p 递归创建目录
mkdir -p build

cd ./build
cmake ..
make -j4

# $? 执行上一个指令的返回值 (显示最后命令的退出状态，0为true，其他均为false)
if [ "$?" -eq '0' ];then  
  echo "cmake complies ok"
else
  echo "cmake complies error"
  exit -1
fi


