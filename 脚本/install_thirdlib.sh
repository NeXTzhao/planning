#!/bin/bash

# 定义变量
OSQP_VERSION="0.4.0"
INSTALL_DIR="/usr/local"  # 安装目录

# 克隆 OSQP 存储库并切换到指定版本
git clone --recursive --branch v${OSQP_VERSION} https://github.com/oxfordcontrol/osqp.git
cd osqp

# 构建和安装 OSQP
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} ..
make
sudo make install

# 验证安装
echo "OSQP 安装完成。版本信息："
osqp_version

# 返回上级目录
cd ../../

echo "安装脚本执行完成。"

