/*************************************************************************************************************************************

Astart改进目录，对A*算法做出优化：加入靠近路沿的启发函数，并对生成的轨迹点做了均值滤波处理，使轨迹更加平滑。

ros目录中，purepursuit功能包使用purepursuit算法对spline生成的样条曲线进行了路径跟踪。lqr_steering功能包使用lqr算法对生成的五次多项式轨迹进行横向路径跟踪。

***************************************************************************************************************************************/

Astart改进:

g++编译:g++ -std=c++11  xxx.cpp -o xx $(pkg-config --cflags --libs opencv) （需要安装opencv）

处理过程：先用opencv将图片做灰度处理，再做二值化，将像素保存到vector二维数组作为地图，设置起点和终点，调用AStart算法(改进版：加入路沿代价函数)找到一条路径，由于算法会导致路径出现锯齿状，故用均值滤波对路径点做平滑处理。

1.原始地图:

![mapload4](https://user-images.githubusercontent.com/68492981/132976491-de0eb792-02cf-4d98-a0cc-24c78338121e.jpg)

2.A*算法生成的路径不平滑且贴近路沿，故增加道路膨胀层并加入靠近路沿的启发函数:

![loadToMap1](https://user-images.githubusercontent.com/68492981/133076047-7c432bd4-a349-4288-8f30-e6b61ddbc2e9.jpg)

![loadToMap4](https://user-images.githubusercontent.com/68492981/132976596-99eee2ee-7b96-464c-9700-36805340588b.jpg)

3.运用均值滤波对路径做平滑处理并加大膨胀半径:

![loadToMap7](https://user-images.githubusercontent.com/68492981/132976579-f1298c8a-17c5-4eeb-8fc4-a1b2bfde91ae.jpg)


ros：（Gazebo仿真）

1.使用Gazebo仿真需要安装的功能包

    sudo apt-get install -y ros-kinetic-gazebo-ros-control 

    sudo apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers

    sudo apt-get install -y ros-kinetic-gazebo-ros-control

2.创建工作空间 catkin_ws

创建src文件，放置功能包源码：

mkdir -p ~/catkin_ws/src

将ros/src下的功能包复制粘贴到创建的目录下
进入src文件夹

cd ~/catkin_ws/src （这一步省略）

初始化文件夹

catkin_init_workspace

编译工作空间 catkin_make

cd ~/catkin_ws/

catkin_make

3.算法实现

3.1 pure_pursuit算法：

实现思路：
1. 运用spline插值进行简单轨迹生成
2. 编写Pure_Pursuit纯路径跟踪算法，对生成的轨迹进行跟踪

操作步骤：编译完成之后source devel/setup.sh 依次启动以下节点

    roslaunch car_model spawn_car.launch
    
    roslaunch purepursuit splinepath.launch 
    
    roslaunch purepursuit purepursuit.launch
    
    rviz 中add /splinepoints /rvizpath  /smart
![purepursuit](https://user-images.githubusercontent.com/68492981/138063800-ce4cab93-26f3-41c9-a0cc-80469628dde1.png)


3.2 lqr横向控制算法：

实现思路：
1. 运用五次多项式生成控制算法所需要的轨迹
2. 编写lqr路径跟踪算法，对轨迹进行横向跟踪控制

操作步骤：编译完成之后source devel/setup.sh 依次启动以下节点

    roslaunch car_model spawn_car.launch
   
    roslaunch lqr_steering frenet_lqr.launch 
    
    rviz add /trajector_ypath /rviz_path  /smart
![lqr](https://user-images.githubusercontent.com/68492981/138063725-e4de2f2c-9bdb-4e41-8c4c-d6c222d00687.png)

