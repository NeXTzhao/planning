# 1 目录概述

***routing_planning/ros/src***

  ros工作空间中，purepursuit功能包使用purepursuit算法对spline生成的样条曲线进行了路径跟踪。lqr_steering功能包使用lqr算法对生成的五次多项式轨迹进行横向路径跟踪。

## 2 算法介绍

### ***2.2 ROS（Gazebo仿真）***

```shell
系统要求：ubuntu、ros、gazebo
```
### 2.2.1 使用Gazebo仿真需要安装的功能包


```shell
sudo apt-get install -y ros-kinetic-gazebo-ros-control
sudo apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers
sudo apt-get install -y ros-kinetic-gazebo-ros-control
```

### 2.2.2 创建工作空间 catkin_ws

```shell
1.创建src文件，放置功能包源码：
  mkdir -p ~/catkin_ws/src

2.进入src文件夹
  cd ~/catkin_ws/src

3.将路径ros/src下的功能包复制粘贴到创建的src目录下

4.初始化文件夹
  catkin_init_workspace

5.编译工作空间catkin_make
  cd ~/catkin_ws/
  catkin_make
```

### 2.2.3 Pure_pursuit算法

**实现思路：**

1. 运用spline插值进行简单轨迹生成
2. 编写pure_pursuit纯路径跟踪算法，对生成的轨迹进行跟踪

**操作步骤：（新开终端窗口）**

```shell
source devel/setup.sh
roslaunch car_model spawn_car.launch
roslaunch purepursuit splinepath.launch 
roslaunch purepursuit purepursuit.launch
rviz 中add /splinepoints /rvizpath  /smart（在rviz显示中，红色为小车运动轨迹，绿色为规划模块给出的轨迹）
```

**Pure_pursuit仿真结果：**
![pure_pursuit](https://user-images.githubusercontent.com/68492981/138196938-870ce64e-2e64-4948-b9ae-9c8617924550.png)


### 2.2.4 LQR横向控制算法

**实现思路：**

1. 运用五次多项式生成控制算法所需要的轨迹
2. 编写lqr路径跟踪算法，对轨迹进行跟踪控制

**操作步骤：（新开终端窗口）**

```shell
source devel/setup.sh
roslaunch car_model spawn_car.launch
roslaunch lqr_steering frenet_lqr.launch 
rviz 中add /trajector_ypath /rviz_path  /smart　（在rviz显示中，红色为小车运动轨迹，绿色为规划模块给出的轨迹）
```

**LQR仿真结果：**
![lqr](https://user-images.githubusercontent.com/68492981/138196914-fa28cc20-9e5a-4300-adde-77ffc9852b69.png)



