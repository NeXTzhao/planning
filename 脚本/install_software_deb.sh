#!/bin/bash

# 指定.deb文件所在的文件夹路径
deb_folder="~/Downloads/software"

# 进入.deb文件所在的文件夹
cd "$deb_folder"

# 遍历.deb文件并进行安装
for deb_file in *.deb; do
    sudo dpkg -i "$deb_file"
done

echo "all installed"
