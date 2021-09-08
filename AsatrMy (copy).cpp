#include <iostream>
#include <fstream>
#include <string>

#include <vector>
#include <list>

#include <cstddef>
#include <typeinfo>
#include <time.h>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define low 400
#define row 400
// #define filePath

using namespace std;

const int kCost1 = 10; //直移一格消耗
const int kCost2 = 14; //斜移一格消耗

//定义每个点的结构体,这个点包含自身坐标(x,y),FGH值,父节点
struct Point
{
    int x, y;      //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
    int F, G, H;   //F=G+H
    Point *parent; //parent的坐标
    //初始化变量
    Point(int x, int y)
    {
        this->x = x;
        this->y = y;
        this->F = 0;
        this->G = 0;
        this->H = 0;
        this->parent = nullptr;
    }
    Point(int x, int y, Point *parent)
    {
        this->x = x;
        this->y = y;
        this->F = 0;
        this->G = 0;
        this->H = 0;
        this->parent = parent;
    }
};

list<Point *> openList;  //开启列表,还未探索
list<Point *> closeList; //关闭列表,被排除的点,也就是被归为父节点的点

vector<vector<int>> maze(row, vector<int>(low)); //定义一个矩阵，用于存放数据

// 计算G值:可以斜着走,计算起始点到中转点的距离
int calcG(Point *temp_start, Point *point)
{
    //斜着走向走距离 > 1 -->KCost2=14 ;
    int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
    //判断父节点是否存在,初始节点的父节点为空
    int parentG = (point->parent == nullptr) ? 0 : point->parent->G;
    return parentG + extraG;
}

// 计算H值:只能横纵向移动,计算中转点到终点的距离
int calcH(Point *point, Point *end)
{
    //用简单的欧几里得距离计算H，这个H的计算是关键
    return sqrt((double)(end->x - point->x) * (double)(end->x - point->x) + (double)(end->y - point->y) * (double)(end->y - point->y)) * kCost1;
}

// 计算F值,也就是
int calcF(Point *point)
{
    return point->G + point->H;
}

//在周围列表中(open列表)中返回最小值
Point *getLeastFpoint()
{
    if (!openList.empty())
    {
        auto resPoint = openList.front();
        for (auto &point : openList)
        {
            if (point->F < resPoint->F)
                resPoint = point;
        }
        return resPoint;
    }
    return nullptr;
}

// 定义这个节点是否在open或者close列表中
Point *isInList(const list<Point *> &list, const Point *point)
{
    //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
    for (auto p : list)
    {
        if (p->x == point->x && p->y == point->y)
            return p;
    }
    return nullptr;
}

// 判断是否能够被搜索
bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner)
{
    // 如果点与当前节点重合、超出地图、是障碍物(坐标点为1的点)、或者在关闭列表中，返回isIgnoreCorner
    if (target->x < 0 || target->x > maze.size() - 1 || target->y < 0 || target->y > maze[0].size() - 1 || maze[target->x][target->y] == 1 || target->x == point->x && target->y == point->y || isInList(closeList, target))
    {
        return isIgnoreCorner;
    }
    else
    {
        if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以
        {
            return true;
        }
        else
        {
            // 不能沿着障碍物底边斜着穿过去
            return (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0) ? true : isIgnoreCorner;
        }
    }
}

// 寻找父节点周围的点
vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner)
{
    vector<Point *> surroundPoints;

    for (int x = point->x - 1; x <= point->x + 1; x++)
        for (int y = point->y - 1; y <= point->y + 1; y++)
            if (isCanreach(point, new Point(x, y), isIgnoreCorner))
                // 加入数组末尾
                surroundPoints.push_back(new Point(x, y));

    return surroundPoints;
}

/*
    核心代码:寻找最短路径点
*/
Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
    // 从起点A开始, 把它作为待处理的方格存入一个"开启列表", 开启列表就是一个等待检查方格的列表.
    openList.push_back(new Point(startPoint.x, startPoint.y));
    while (!openList.empty())
    {
        auto curPoint = getLeastFpoint(); //找到F值最小的点

        // 核心列表:不断把当前搜寻到的点归类
        //从开启列表中删除
        openList.remove(curPoint);
        //放到关闭列表
        closeList.push_back(curPoint);

        //1,找到当前周围八个格中可以通过的格子
        auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
        for (auto &target : surroundPoints)
        {
            //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
            // 判断完周围点后,如果没有查询过就加入到open中
            if (!isInList(openList, target))
            {
                target->parent = curPoint;

                target->G = calcG(curPoint, target);
                target->H = calcH(target, &endPoint);
                target->F = calcF(target);

                openList.push_back(target);
            }
            //3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
            // 如果某个相邻方格D已经在 "开启列表" 里了, 检查如果用新的路径 (就是经过C 的路径) 到达它的话, G值是否会更低一些, 如果新的G值更低, 那就把它的 "父方格" 改为目前选中的方格 C, 然后重新计算它的 F 值和 G 值 (H 值不需要重新计算, 因为对于每个方块, H 值是不变的). 如果新的 G 值比较高, 就说明经过 C 再到达 D 不是一个明智的选择, 因为它需要更远的路, 这时我们什么也不做.
            else
            {
                int tempG = calcG(curPoint, target);
                if (tempG < target->G)
                {
                    target->parent = curPoint;

                    target->G = tempG;
                    target->F = calcF(target);
                }
            }
            Point *resPoint = isInList(openList, &endPoint); //判断终点是否在列表中
            if (resPoint)
                return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
        }
    }
    return nullptr;
}

// 用链表存放路径,因为链表的插入和删除很快,缺点是查询慢,当我们这里不需要查询
list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
    Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
    list<Point *> path;
    //返回路径，如果没找到路径，返回空链表
    while (result)
    {
        // 插入到头部
        path.push_front(result);
        result = result->parent;
    }

    // 清空临时开闭列表，防止重复执行GetPath导致结果异常
    openList.clear();
    closeList.clear();

    return path;
}

// 封装读取图片信息的方法
void imageChange()
{
    using namespace cv;
    string fileName = "/home/next/ros_workspace/map_file/AStart/mapload1.jpg";
    // cout << filePath << endl;
    // string a="";
    // string fileName = " " " + filePath + " " ";
    // string fileName = filePath;
    cv::Mat img = imread(fileName);
    if (img.empty())
    {
        cout << "读取文件失败";
    }
    int row1 = img.rows;
    int low1 = img.cols;
    cv::namedWindow("原图");
    cv::imshow("原图", img);

    cv::Mat gray;
    cv::cvtColor(img, gray, COLOR_BGR2GRAY); //先转为灰度图

    cv::Mat mapNew;
    cv::threshold(gray, mapNew, 127, 255, THRESH_BINARY); //二值化阈值处理
    cv::imshow("gray_image", mapNew);

    // 保存像素信息的txt文本路径
    // fstream file("mapPointsRead.txt", ios::out);
    // stringstream ss;
    // string data0;

    fstream file("mapPointsShow.txt", ios::out);
    stringstream ss;
    string data;

    for (int i = 0; i < row1; i++)
    {
        for (int j = 0; j < low1; j++)
        {
            maze[i][j] = (mapNew.at<uchar>(i, j) == 255) ? 0 : 1; //img的矩阵数据传给二维数组maze[][]
            int value = (mapNew.at<uchar>(i, j) == 255) ? 0 : 1;
            ss << dec << value;
            ss >> data;
            ss.clear();
            file << data;
        }
        file << endl;
    }
    waitKey(1000);
}

int main()
{

    // imageChange

    // ifstream infile;                                                       //定义读取文件流
    // infile.open("/home/next/ros_workspace/map_file/AStart/mapPoints.txt"); //打开文件

    // for (int i = 0; i < row; i++)
    // {
    //     for (int j = 0; j < low; j++) //定义列循环
    //     {
    //         infile >> maze[i][j]; //读取一个值（空格、制表符、换行隔开）就写入到矩阵中，行列不断循环进行
    //     }
    // }
    // infile.close(); //读取完成之后关闭文件
    cout << "开始处理图片" << endl;
    imageChange();
    cout << "图片处理完成" << endl;

    // 输出二维动态数组
    for (int i = 0; i < maze.size(); i++)
    {
        for (int j = 0; j < maze[i].size(); j++)
        {
            cout << maze[i][j];
        }
        cout << "\n";
    }
    cout << "A*算法地图导入完成" << endl;
    //设置起始和结束点
    Point start(0, 0);
    Point end(399, 399);

    cout << "A*算法开始寻找路径" << endl;

    clock_t t;   //定义时钟变量t
    t = clock(); //调用前时间
    //A*算法找寻路径
    list<Point *> path = GetPath(start, end, false);
    t = clock() - t;                                     //计算函数耗时
    printf("time=%lf s\n", ((float)t) / CLOCKS_PER_SEC); //转化为秒

    cout << "A*算法寻路成功" << endl;

    // 打印
    for (auto &p : path)
    {
        // cout << '(' << p->x << ',' << p->y << ')' << endl;
        maze[p->x][p->y] = 4;
    }

    // 保存寻路之后的地图  fstream有两个子类：ifstream(input file stream)和ofstream(outpu file stream)
    fstream file("loadPoints.txt", ios::out);

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < low; j++)
        {
            file << maze[i][j];
        }
        file << endl;
    }
    file.close();

    return 0;
}
