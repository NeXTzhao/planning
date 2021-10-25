/*==================================================================================================================*/

velodyne激光雷达 & slam建图 & NDT定位 & pcl库

1. 三维激光雷达建2D图 gmapping操作步骤：
    1.roslaunch velodyne_pointcloud VLP16_points.launch 
    2.rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link velodyne 0
    3.rviz -f velodyne
    4.rosrun gmapping slam_gmapping
    5.rosrun laser_scan_matcher laser_scan_matcher_node _fixed_frame:=odom
    6.rosrun rqt_tf_tree rqt_tf_tree
    7.rosrun map_server map_saver  map:=/projected_map -f ~/map

    camera_init

2. A_LOAM建图操作步骤：
    1.运行vlp
    roslaunch velodyne_pointcloud VLP16_points.launch
    2.运行A-LOAM
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    3.保存二維柵格地圖：（節點是/projected_map）（這個地圖格式可以使用到gampping）
    rosrun map_server map_saver  map:=/projected_map -f ~/map

3. Ａ_LOAM建圖同時構建柵格地圖：
    1.建立launch启动文件，需要单独建立一个包
        创建launch文件
        src目录下 
            catkin_create_pkg xxx(文件名)
    2.在rviz 中添加一個“OccupancyMap” 模块. 设置topic 为"/octomap_full"
    3.rosbag record /"your topic"

4. Lego_loam:
    运行lego_loam 播放bag文件需要加 --clock --topic /velodyne_points
    在rviz中要点击Map cloud 不然不会保存pcd文件


5. NDT定位算法实现
    1.弧度转角度:()*180/M_pi
    2.tf坐标:
        <node pkg="tf2_ros" type="static_transform_publisher" name="localizer_to_base_link" args="0 0 1.4 0 0 0 base_link velodyne"/>
        <node pkg="tf2_ros" type="static_transform_publisher" name="world_to_map" args="0 0 0 0 0 1.57 world map"/>
    3.urdf可以用默认参数
    4.rviz显示一定要用2D pose Estimate给模型一个初始的姿态

/*==================================================================================================================*/

VSCODE

1. 插件
    代码格式化:Clang-Format
        需要在终端sudo apt-get install clang-format





/*==================================================================================================================*/

ROS

1. ROS 系统中存在的 TF 变换：
    rosrun rqt_tf_tree rqt_tf_tree

    ROS查看发布者和订阅者：
     rostopic list -v

2. 编写launch文件要注意的地方：
    <launch>
        <node pkg="运行catkin_creat_pkg时创建的包名" type="CMakeLists.txt中编写的add_executable(xx src/xx.cpp) 中命名的xx可执行文件" name="这里填写的名称会覆盖掉init()中命名的节点名称"/>
    </launch>
    
3. roslaunch gazebo_ros empty_world.launch 

4. rosmsg show geometry_msgs/Twist
    geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
    geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z


    sudo apt-get remove gazebo-*
相关报错处理
1. ros中自定义的msg需要修改的文件:
    CMakelist.txt 里面要添加
        find_package(
        catkin REQUIRED COMPONENTS
            message_generation  //这一个消息
        )
    package.xml 和运行时依赖要添加编译依赖
        <build_depend>message_generation</build_depend>
        <exec_depend>message_runtimes</exec_depend>

2. ros-kinetic包catkin_make报错：Could not find a package configuration file provided by "can_msgs" with any of the following names:
    排查步骤：
    1.rosmsg list
        查看所有的msg消息
    2.如果确认没有安装can_msgs,输入如下指令安装：
        sudo apt-get install ros-kinetic-can-msgs
    3.碰到其他类似缺少安装包，但是又不知道安装包叫什么的时候，可输入如下指令查询：
        apt-cache search ros-kinetic

3. catkin_make的时候出现：fatal error: xxx.h: 没有那个文件或目录
    解决：检查CMakeLists.txt 这几行
        include_directories(
            #include
            ${catkin_INCLUDE_DIRS}
            )
        去掉include前面的注释，然后要写成include_directories(include ${catkin_INCLUDE_DIRS})这种形式，不能换行，同时.cpp文件中只用些.h文件的名称，不要在前面加上包名
4. 程序“roscore”尚未安装
    终端gedit ~/.bashrc
    查看是否还有“source /opt/ros/kinetic/setup.bash”这一行，如果有，那就在终端在source ~/.bashrc

/*==================================================================================================================*/

Eigen库

1. eigen安装见csdn，安装完成之后要使用，需添加头文件#include <Eigen/Eigen>

2. 
    Eigen通过tyepdef定义了许多内置类型，不过底层仍然是Eigen::Matrix,如下所示：
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero():     //初始化为0       //Matrix3d实质上是Eigen::Matrix<double, 3, 3> 
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;     //如果不确定矩阵大小，可以使用动态大小的矩阵
    Eigen::MatrixXd matrix_xd;                                                //与上类似，表示任意大小的元素类型为double的矩阵变量
    Eigen::MatrixXf matrix_xf;                                                //表示任意大小的元素类型为float的矩阵变量
    Eigen::Vector3d v_3d;                                                     //Vector3d实质上是Eigen::Matrix<double, 3, 1>   三行一列

3. 
    3.1 加减乘除跟数字操作一样
    3.2 赋值操作:
        matrix_33<<1,2,3,4,5,6,7,8,9;
        或 matrix_33(0,0)=1;
    3.3
    cout << matrix_33.transpose() << endl;    //转置
	cout << matrix_33.sum() << endl;          //各元素和
	cout << matrix_33.trace() << endl;        //迹
	cout << matrix_33 * 10 << endl;           //数乘
	cout << matrix_33.inverse() << endl;      //逆
	cout << matrix_33.determinant() << endl;  //行列式
    cout << matrix_33.adjoint() << endl;      //伴随
    cout << matrix_33.cwiseAbs() << endl;     //绝对值
    cout << matrix_33.maxCoeff() << endl;     //求最大系数

    3.4
    Eigen::Vector2d C = A.colPivHouseholderQr().solve(b); // 求解Ax=b ; Vector2d => Matrix<double,2, 1>
    cwiseAbs()求绝对值、maxcoff()求最大系数

    3.5 设置单位矩阵
        Eigen::MatrixXd::Identity(4, 4)  
        或
        Eigen::Matrix<double ,4,4> A;
        A.setIdentity(4, 4);


/*==================================================================================================================*/

C/C++

1. printf("% ",xx)输出
    %d整型输出，％ld长整型输出，
    %p指针变量地址，如果数处数据不够8位数，则左边补零
    %o以八进制数形式输出整数，
    ％x以十六进制数形式输出整数，
    ％u以十进制数输出unsigned型数据(无符号数)。
    ％c用来输出一个字符，
    ％s用来输出一个字符串，
    ％f用来输出实数，以小数形式输出，
    ％e以指数形式输出实数，
    ％g根据大小自动选f格式或e格式，且不输出无意义的零

2. CMakeLists.txt编写
    # CMake最低版本要求
    cmake_minimum_required(VERSION 3.5)

    # 添加c++11标准支持
    set(CMAKE_CXX_FLAGS "-std=c++11" )

    # 项目名称(任意)
    project(AStar)

    # 设置文件夹,添加头文件
    include_directories(${AStart} AStar.h)

    # 生成可执行文件 AStart可执行文件名称(项目名)  AStart.cpp是源文件名称,还有其他的可以在后面添加
    add_executable(AStar AStar.cpp main.cpp)

3.  c++语法
    匿名函数：
        C++中的匿名函数通常为[capture](parameters)->return-type{body}，
        当parameters为空的时候，()可以被省去，当body只有“return”或者返回为void，那么”->return-type“可以被省去

    %取余和std::fmod()的区别：
        double c=a%b;  //此方式只能用于int

        // 将浮点数a分解成整数部分和小数部分，返回小数部分，将整数部分存入b所指内存中，
        // 计算a/b的余数，返回a-n*b，符号同a。n=[a/b](向离开零的方向取整)。
        double c1=fmod(a,b); 

    inline用法:
        在c/c++中，为了解决一些频繁调用的小函数大量消耗栈空间（栈内存）的问题，
        特别的引入了inline修饰符，表示为内联函数，栈空间就是指放置程序的局部数据（也就是函数内数据）的内存空间

    using State = std::array<double, 3>;
    using Condition = std::pair<State, double>;
        using 可以用作声明
        std::pair主要的作用是将两个数据组合成一个数据，两个数据可以是同一类型或者不同类型
        pair实质上是一个结构体，其主要的两个成员变量是first和second，这两个变量可以直接使用。

    std::vector的insert用法：
        v.insert(v.begin(),8);//在最前面插入新元素,此时v为8 2 7 9 5
        v.insert(v.begin()+3,1);//在迭代器中下标为3的元素前插入新元素,此时v为8 2 7 1 9 5
        v.insert(v.end(),3);//在向量末尾追加新元素,此时v为8 2 7 1 9 5 3
        v.insert(v.end(),3,0);//在尾部插入3个0,此时v为8 2 7 1 9 5 3 0 0 0

    std::pow(x, y)计算x的y次幂


4. 语法小技巧
4.1 break 跳出多层循环 
    for (i = 0; i < n; i++) {
            for (j = 0; j < m; j++) {
            // 这里执行相应的程序任务
            if (i == 4 && j == 3) break; // 制造一个退出循环的条件
            }
        if(j != m) break; // 如果 j != m ，那么证明在内层 for 循环中，还没有完成 m 次循环就被打断
    }


相关报错处理
1. 包含 OpenCv 库和支持 c++11 的cpp文件g++编译
    g++ -std=c++11  xxx.cpp -o xx $(pkg-config --cflags --libs opencv)

2. 编译报错ERROR: cannot launch node of type [robot_vision/motion_detector.py]: can not locate node [motion_detector.py] in package [robot_vision]
    解决：报错是因为权限不够！需要把xxx.py改成可执行文件权限

/*==================================================================================================================*/

Git & GitHub

1. 基础命令的含义；
    clone       克隆远程仓库
    init        初始化仓库
    remote      连接远程仓库
    pull        从远程仓库下拉获取新数据
    push        将本地仓库新增或修改文件上传到远程仓库
    add         添加文件或者修改文件，commit以及push之前使用
    log         当前仓库提交过的日志信息
    status      当前仓库版本状态
    commit      提交到当前仓库中
    branch      分支命令，相关增删查操作
    checkout    使用远程仓库最后一个版本完全覆盖当前仓库内容／选择分支branch
    diff        对比版本内容
    merge       合并版本内容

2. 提交代码 
    向本地仓库提交代码(终端操作)
        git init    初始化git仓库
        git status  查看文件状态
        git add     文件列表 追踪文件
        git commit -m 提交信息 向仓库中提交代码（原则：每次提交只包含一个功能，不要再一次提交中，包含多个功能或功能中还包含bug，不利于后期恢复项目的状态）
        git log     查看提交记录

    向github中提交代码(终端操作)
        echo "# grq" >> README.md
        git init
        git add README.md
        git commit -m "first commit"
        git branch -M main
        git remote add origin git@github.com:NeXTzhao/grq.git（origin 为远程链接的别名）
        git push -u origin main
        
3. vscode + CMake 调试
        launch.json注释
        /*
        
        {
            "version": "0.2.0",
            "configurations": [
                {
                    "name": "(gdb) Launch", // 配置名称，将会在启动配置的下拉菜单中显示
                    "type": "cppdbg", // 配置类型，这里只能为cppdbg
                    "request": "launch", // 请求配置类型，可以为launch（启动）或attach（附加）
                    "program": "${fileDirname}/${fileBasenameNoExtension}.exe", // 将要进行调试的程序的路径
                    "args": [], // 程序调试时传递给程序的命令行参数，一般设为空即可
                    "stopAtEntry": false, // 设为true时程序将暂停在程序入口处，我一般设置为true
                    "cwd": "${workspaceFolder}", // 调试程序时的工作目录
                    "environment": [], // （环境变量？）
                    "externalConsole": true, // 调试时是否显示控制台窗口，一般设置为true显示控制台
                    "internalConsoleOptions": "neverOpen", // 如果不设为neverOpen，调试时会跳到“调试控制台”选项卡，你应该不需要对gdb手动输命令吧？
                    "MIMode": "gdb", // 指定连接的调试器，可以为gdb或lldb。但目前lldb在windows下没有预编译好的版本。
                    "miDebuggerPath": "gdb", // 调试器路径，Windows下后缀不能省略，Linux下则去掉
                    "setupCommands": [ // 用处未知，模板如此
                        {
                            "description": "Enable pretty-printing for gdb",
                            "text": "-enable-pretty-printing",
                            "ignoreFailures": false
                        }
                    ],
                    "preLaunchTask": "Compile" // 调试会话开始前执行的任务，一般为编译程序。与tasks.json的label相对应
                }
            ]
        }

        */


相关报错处理：
1. vscode 上传代码报错：Failed to connect to github.com port 443:connection timed out
    (知识：端口 443，基于TLS/SSL的http协议，此端口用于安全的 Web 浏览器通信。通过此类连接传输的数据具有很强的抗窃听和拦截能力。
    此外，可以非常自信地验证远程连接服务器的身份。提供接受和建立安全连接的 Web 服务器在此端口上侦听来自需要强大通信安全性的 Web 浏览器的连接。
    建立后，Web 浏览器会通过在其窗口的状态区域中显示图标（挂锁、未损坏的钥匙等）来通知用户这些安全连接。
    相关端口：80，81，82，8080，8090)

    解决：这是由于开启了vpn全局代理,导致代理端口号出问题，解决办法跟Ubuntu 终端翻墙一样，把自己的在代理器中的ip和端口添加到配置文件，
        这一步之前需要把Ubuntu设置中的NetWork proxy设置成手动，以此设置http https socks的ip和端口
        next@next:~/ros_workspace/map_file/AStart$ git config --global http.proxy http://127.0.0.1:1087 
        next@next:~/ros_workspace/map_file/AStart$ git config --global https.proxy http://127.0.0.1:1087
        next@next:~/ros_workspace/map_file/AStart$ git config --global all.proxy socks5://127.0.0.1:8081

2. git clone慢该怎么办？ 
    安装 Linux 的软件安装管理实用程序stow, sudo apt-get install stow

3. 有时候git clone下来的文件，在编译的时候无法成功，用直接下载下来的源码（选择了branches）反而能行？
    git clone 默认下载主线的代码，所以当有版本需要求的时候要注意指定版本
    git clone --branch [tags标签] [git地址] 或者 git clone --b [tags标签] [git地址]

4. git忽略规则.gitignore不生效
    解决:先把本地缓存删除（改变成未track状态），然后再提交。
        git rm -r --cached .
        git add .
        git commit -m 'update .gitignore'
        然后再次推送
        
5. vscode终端无法启动 报路径不存在问题（例如安装或卸载zsh之后出现 /bin/zsh 不存在）
    解决：修改vscode的默认shell 解决这个问题：
        在 VSCode 中打开设置搜索 Cntr + Shift + p
        搜索 default
        点击 Terminal: Select Default Shell
        单击zsh或bash  /usr/bin/zsh，我选择了zsh，也可以使用其他终端选项。


        
/*==================================================================================================================*/

Shell

1.  shell脚本配置Ubuntu开发环境 & 软件安装
        Ubuntu开发环境
            #!/bin/bash
                echo -e "\033[45;37m=========TSARI 开发环境配置==========\033[0m"
                echo -e "\033[45;37m---------更新资源---------\033[0m"
                sudo apt-get -y update
                sudo apt-get -y upgrade

                ...
                (未完待续...)

        软件安装 (优化写法)：
            install_software() {
                #install git
                if (echo "${install_software_list[@]}" | grep -wq "git");then
                    apt-get -y install git && echo -e "${BLUE_COLOR}git install completed.${END_COLOR}" 
                fi
            }
  


/*==================================================================================================================*/

Linux & Ubuntu 

1. 界面美化:gonme look
    1.1 美化终端: 安装zsh替换shell,在gedit ~/.zsher中查找ZSH_THEME关键字,修改引号里面的主题名 （ /* https://ohmyz.sh/#install　*/ 这个网站里面查找好看的主题）

2. 常用快捷键 & 命令
    2.1 快捷键
        关闭终端快捷键：ctrl+shift+q
        同一个终端开多个窗口：ctrl+shift+t
        vscode 打开底部终端：Ctrl + ～ 
    
    2.2 命令
        目录结构
        整个Linux的根目录 /
        用户目录的根目录 ~ ： 在 /home/gec （gec即用户目录，不同用户有不同的用户目录）
        cd ~ 或者 cd /home/gec 都可以
        *help: 第一个 / 表示绝对路径，即从根目录开始
        （命令后面可以加参数，获取更多需要的信息，也可以多个参数同时用，如 ls -al具体网上搜索）
        命令可以寻求帮助， 命令 --help
        如 ls --help
        ~help：文件名自动补全功能，输入一半的文件名，再按Tab键
        
        （1）目录信息查看命令 ls
        （2）目录切换命令 cd
        （3）当前路径显示命令 pwd
        （4）系统信息查看命令 uname
        （5）清理屏幕命令 clear / c
        （6）显示文件内容命令 cat
        （7）切换用户身份命令 sudo
        （8）文件拷贝命令 cp
        （9）切换用户命令 su
            sudo su
            会将用户改为root用户，不建议用此方法运行（ rm / -rf 即删库）
            切回用户名： sudo su 用户名
        （10）移动文件命令（或者将文件名重命名） mv
            重命名： mv a.c b.c 实现将a.c文件名改为b.c
            将a.c移动到test/目录： mv a.c test/
        （11）创建文件夹命令 mkdir
        （12）创建文件命令 touch
        （13）删除命令 rm
        （14）目录删除命令 rmdir
        （15）显示网络配置信息命令 ifconfig
            查看有多少网卡：ifconfig -a
            打开eth0网卡：ifconfig eth0 up (最好加权限 sudo)
            关闭eth0网卡：ifconfig eth0 down (最好加权限 sudo)
            重启网卡： ifconfig eth0 reload
            更改ip地址：sudo ifconfig eth0 地址
        （16）重启（系统）命令 reboot
        （17）关机（系统）命令 poweroff
        （18）系统帮助命令 man
            man printf 即可搜到printf的信息（可加参数）
            按q 退出
        （19）数据同步写入磁盘命令 sync
        （20）查找文件命令 find （参数很多）
            首先 touch a.c (在test/ 目录下)
            find -name a.c  即可找到该文件的位置
            结果是 ./test/a.c ( ./ 表示当前目录的意思)
        （21）查找内容命令 grep
            在根目录下哪些文件有Ubuntu字样：
            grep -nr “Ubuntu” /
        （22）文件夹大小查看命令 du
            如查看test/目录下文件大小
            du -sh test/
        （23）磁盘空间检查命令 df
        （24）使用gedit打开某个文件命令 gedit
            gedit a.c
        （25）当前的系统进程查看命令 ps
        （26）进程实时运行状态查看命令 top
            q 退出
        （27）文件类型查看命令 file
            进入共享文件夹目录：cd /mnt/hgfs/共享文件夹名

3. 终端实现翻墙：
    终端默认不动，手动配置,前提是要修改后NetWork中的配置
    修改 sudo gedit ~/.bashrc
        export https_proxy=http://127.0.0.1:{http port};
        export http_proxy=http://127.0.0.1:{http port};
        export all_proxy=socks5://127.0.0.1:{socks port}

4. 终端常用小工具
    4.1 终端写入窗口打印出的信息到文件中
        先运行script -a xx.txt  再运行你的执行文件  最后再运行script -a xx.txt

5. 资源库常用命令：
    sudo apt-get update  //更新源 
    sudo apt-get install package  //安装包 
    sudo apt-get remove package  //删除包 
    sudo apt-cache search package  //搜索软件包
    sudo apt-cache show package  //获取包的相关信息，如说明、大小、版本等 
    sudo apt-get install package --reinstall  //重新安装包 
    sudo apt-get -f install  //修复安装 
    sudo apt-get remove package --purge  //删除包，包括配置文件等 
    sudo apt-get build-dep package  //安装相关的编译环境 
    sudo apt-get upgrade  //更新已安装的包 
    sudo apt-get dist-upgrade  //升级系统 
    sudo apt-cache depends package  //了解使用该包依赖那些包 
    sudo apt-cache rdepends package  //查看该包被哪些包依赖 
    sudo apt-get source package  //下载该包的源代码 
    sudo apt-get clean && sudo apt-get autoclean  //清理无用的包 
    sudo apt-get check  //检查是否有损坏的依赖

相关报错处理
1. sudo apt-get update报错 E: 部分索引文件下载失败。如果忽略它们，那将转而使用旧的索引文件。
     解决：进入到 cd /etc/apt/sources.list.d/ 再ls查看索引, 然后删除对应的报错索引

/*==================================================================================================================*/
