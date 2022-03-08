#!/bin/bash
###
 # @Author: your name
 # @Date: 2022-03-8 
 # @LastEditTime: 2022-03-8
 # @LastEditors: Please set LastEditors
 # @Description: In User Settings Edit
 # @FilePath: /undefined/home/next/ros_workspace/routing_planning/shell脚本/ubuntu_env_install.sh
### 

echo -e "\033[45;37m=========安装VTD开发环境依赖==========\033[0m"
echo -e "\033[45;37m---------更新资源---------\033[0m"
  sudo apt-get -y update
  sudo apt-get -y upgrade
echo -e "\033[45;37m---------开始安装VTD依赖库---------\033[0m"
  sudo apt install -y xterm
  sudo apt install -y freeglut3 
  sudo apt install -y openssh-server 
  sudo apt install -y nfs-common 
  sudo apt install -y mesa-utils 
  sudo apt install -y xfonts-75dpi 
  sudo apt install -y libusb-0.1-4 
  sudo apt install -y python3 
  sudo apt install -y mesa-common-dev 
  sudo apt install -y libgl1-mesa-dev 
echo -e "\033[45;37m---------VTD依赖库安装完成---------\033[0m"

echo -e "\033[45;37m---------再次更新资源---------\033[0m"
  sudo apt -y update
  sudo apt -y upgrade