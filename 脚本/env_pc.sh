#!/bin/bash

# 定义需要安装的软件包列表
packages=(
    glogg
    terminator
    git
    clang-format
    python3-pip
    htop
    libeigen3-dev
    vlc
    inkscape
)

# 更新软件包列表
sudo apt update

# 安装软件包
for package in "${packages[@]}"
do
    echo "安装$package"
    sudo apt install -y "$package"
done
echo "软件安装完成。"

echo "配置git"
git config --global user.name " "
git config --global user.email " "
git config --global credential.helper store
pip config set global.index-url <https://mirrors.aliyun.com/pypi/simple/>

echo "配置git完成"
