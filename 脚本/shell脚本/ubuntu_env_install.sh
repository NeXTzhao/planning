#!/bin/bash
###
 # @Author: your name
 # @Date: 2021-09-12 16:48:00
 # @LastEditTime: 2021-09-27 22:08:54
 # @LastEditors: Please set LastEditors
 # @Description: In User Settings Edit
 # @FilePath: /undefined/home/next/ros_workspace/routing_planning/shell脚本/ubuntu_env_install.sh
### 
echo -e "\033[45;37m=========TSARI 开发环境配置==========\033[0m"
echo -e "\033[45;37m---------更新资源---------\033[0m"
  sudo apt-get -y update
  sudo apt-get -y upgrade

echo -e "\033[45;37m---------安装boost库---------\033[0m"
  sudo apt-get install -y libboost-all-dev

echo -e "\033[45;37m---------安装zmq库---------\033[0m"
  sudo apt-get install -y libzmq3-dev

echo -e "\033[45;37m---------安装JSON库---------\033[0m"
  sudo apt-get install -y libjsoncpp-dev

# echo -e "\033[45;37m---------安装pip---------\033[0m"
#   sudo apt install -y python-pip

# echo -e "\033[45;37m---------安装pip3---------\033[0m"
#   sudo apt install -y python3-pip

# echo -e "\033[45;37m---------安装glog---------\033[0m"
#   git clone https://github.com/google/glog
#   sudo apt-get install -y autoconf automake libtool
#   CURDIR="`pwd`"
#   echo -e "  \033[47;34m---------进入glog目录---------\033[0m"
#     cd $CURDIR/glog/  
#     CURRENT="`pwd`"
#     echo -e $CURRENT
#     ./autogen.sh
#     ./configure
#     make -j 24
#     sudo make install
#   echo -e "  \033[47;34m---------安装pip glog---------\033[0m"
#   pip install glog
#   echo -e "  \033[47;34m---------安装pip3 glog---------\033[0m"
#   pip3 install glog

echo -e "\033[45;37m---------安装Scipy模块---------\033[0m"
  sudo apt-get install -y python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose

# echo -e "\033[45;37m---------安装PyQt5---------\033[0m"
#   sudo apt-get install -y python-pyqt5

# echo -e "\033[45;37m---------安装pyqtgraph---------\033[0m"
#   sudo pip install pyqtgraph --user

# echo -e "\033[45;37m---------安装matplotlib.pyplot---------\033[0m"
#   sudo pip install matplotlib==2.0.2

echo -e "\033[45;37m---------安装pyzmq---------\033[0m"
  sudo pip install pyzmq

echo -e "\033[45;37m---------安装pcap---------\033[0m"
  sudo apt-get install -y libpcap-dev

echo -e "\033[45;37m---------安装Eigen3---------\033[0m"
  sudo apt-get install -y libeigen3-dev 
  
# echo -e "\033[45;37m---------安装PCL---------\033[0m"
#   echo -e "  \033[47;34m---------安装PCL依赖库---------\033[0m"
#   sudo apt-get install -y git build-essential linux-libc-dev
#   sudo apt-get install -y cmake cmake-gui   
#   sudo apt-get install -y libusb-1.0-0-dev libusb-dev libudev-dev  
#   sudo apt-get install -y mpi-default-dev openmpi-bin openmpi-common    
#   sudo apt-get install -y libflann1.8 libflann-dev  
#   sudo apt-get install -y libeigen3-dev  
#   sudo apt-get install -y libboost-all-dev 
#   sudo apt-get install -y libvtk5.10-qt4 libvtk5.10 libvtk5-dev 
#   sudo apt-get install -y libqhull* libgtest-dev  
#   sudo apt-get install -y freeglut3-dev pkg-config  
#   sudo apt-get install -y libxmu-dev libxi-dev   
#   sudo apt-get install -y mono-complete  
#   sudo apt-get install -y qt-sdk openjdk-8-jdk openjdk-8-jre  
#   echo -e "  \033[47;34m---------安装PCL---------\033[0m"
#   sudo apt-get install libpcl-dev
echo -e "\033[45;37m---------安装ros关节控制相关库---------\033[0m"
  sudo apt-get install -y ros-kinetic-gazebo-ros-control
  sudo apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers
  sudo apt-get install -y ros-kinetic-gazebo-ros-control

echo -e "\033[45;37m---------安装clang-format相关库---------\033[0m"
  sudo apt-get -y install clang-format

  
