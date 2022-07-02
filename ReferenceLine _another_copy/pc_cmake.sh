#!/bin/bash

if [ -d build ];then
  rm -rf build
fi
if [ -d bin ];then
  rm -rf bin
fi
if [ -d lib ];then
  rm -rf lib
fi
if [ -d result_picture ];then
  rm -rf result_picture
fi
# mkdir 有一个参数-p，用于递归创建目录
# mkdir result_picture
mkdir -p build

cd ./build
cmake ..
make -j8

# $? 执行上一个指令的返回值 (显示最后命令的退出状态。0表示没有错误，其他任何值表明有错误)
if [ "$?" -eq '0' ];then
  echo "cmake compiles OK"
  exit 0
else 
  echo "cmake compiles error!!!"
  exit -1
fi
