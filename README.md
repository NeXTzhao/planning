# 1 目录概述

***routing_planning/Astart改进***

  针对A*算法做出优化：加入靠近路沿的启发函数，并对生成的轨迹点做了均值滤波处理，使轨迹更加平滑。

***routing_planning/ros/src***

  ros工作空间中，purepursuit功能包使用purepursuit算法对spline生成的样条曲线进行了路径跟踪。lqr_steering功能包使用lqr算法对生成的五次多项式轨迹进行横向路径跟踪。

## 2 算法介绍

### ***2.1 Astart改进***

```shell
 编译：g++ -std=c++11  xxx.cpp -o xx $(pkg-config --cflags --libs opencv) （需要安装opencv）
```

**实现思路：**

先用opencv将图片做灰度处理，再做二值化，将像素保存到vector二维数组作为地图，设置起点和终点，调用AStart算法(改进版：加入路沿代价函数)找到一条路径，由于算法会导致路径出现锯齿状，故用均值滤波对路径点做平滑处理。

**算法流程：**

1. 原始地图:

<img src="https://user-images.githubusercontent.com/68492981/132976491-de0eb792-02cf-4d98-a0cc-24c78338121e.jpg" alt="mapload4" style="zoom:80%;" />

2. A*算法生成的路径不平滑且贴近路沿，故增加道路膨胀层并加入靠近路沿的启发函数:

<img src="https://user-images.githubusercontent.com/68492981/133076047-7c432bd4-a349-4288-8f30-e6b61ddbc2e9.jpg" alt="loadToMap1" style="zoom: 300%;" />         <img src="https://user-images.githubusercontent.com/68492981/132976596-99eee2ee-7b96-464c-9700-36805340588b.jpg" alt="loadToMap4" style="zoom: 80%;" />

3. 利用均值滤波对路径做平滑处理并加大膨胀半径:

<img src="https://user-images.githubusercontent.com/68492981/132976579-f1298c8a-17c5-4eeb-8fc4-a1b2bfde91ae.jpg" alt="loadToMap7" style="zoom:80%;" />

### ***2.2 ROS（Gazebo仿真）***

```shell
系统要求：ubuntu16.04 + ros-kinetic +gazebo7
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



