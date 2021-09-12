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

using namespace std;

const int kCost1 = 10;    //直移一格消耗
const int kCost2 = 14;    //斜移一格消耗
const int kcost3 = 50000; //在边界一定范围内（也就是kcostmap），cost增加,
const int kconstmap = 3;  //设置膨胀距离
const int sampling = 6;   // 设置采样范围

/*============================================================================*/
/*                                                                            */
/* A*算法实现模块                                                               */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*============================================================================*/

//定义每个点的结构体,这个点包含自身坐标(x,y),FGH值,父节点
struct Point
{
    int x, y;       //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
    int F, G, H, B; //F=G+H
    Point *parent;  //parent的坐标
    //初始化变量
    Point(int x, int y)
    {
        this->x = x;
        this->y = y;
        this->F = 0;
        this->G = 0;
        this->H = 0;
        this->B = 0;
        this->parent = nullptr;
    }
    Point(int x, int y, Point *parent)
    {
        this->x = x;
        this->y = y;
        this->F = 0;
        this->G = 0;
        this->H = 0;
        this->B = 0;
        this->parent = parent;
    }
};

list<Point *> openList;  //开启列表,还未探索
list<Point *> closeList; //关闭列表,被排除的点,也就是被归为父节点的点

// 计算G值:可以斜着走,计算起始点到中转点的距离
int calcG(Point *temp_start, Point *point)
{
    //斜着走向走距离 > 1 -->KCost2=14
    int extraG = ((abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1) ? kCost1 : kCost2;
    //判断父节点是否存在,初始节点的父节点为空
    int parentG = (point->parent == nullptr) ? 0 : point->parent->G;
    return parentG + extraG;
}

// 计算H值:只能横纵向移动,计算中转点到终点的距离
int calcH(Point *point, Point *end)
{
    //欧几里得距离计算H，这个H的计算是关键
    // return sqrt((double)(end->x - point->x) * (double)(end->x - point->x) + (double)(end->y - point->y) * (double)(end->y - point->y)) * kCost1;

    // 由于欧几里得距离的sqrt运算开销过大，另外道路的转换一般只存在四个方向的变换，故采用Manhattan评价函数
    return (abs((double)(end->x - point->x)) + abs((double)(end->y - point->y))) * kCost1;
}

//计算p值：表示离车道距离的代价，距离路沿越近代价越高，这里只有150 和 0两个档位,150表示可以走但是cost会加大
int calcB(const Point *target, vector<vector<int>> array)
{
    return (array[target->x][target->y] == 150) ? kcost3 : 0;
}

// 计算F值,也就是
int calcF(Point *point)
{
    return point->G + point->H + point->B;
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
            {
                resPoint = point;
            }
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
bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner, vector<vector<int>> array)
{
    // 如果点与当前节点重合、超出地图、是障碍物(坐标点为0的点)、或者在关闭列表中，返回isIgnoreCorner
    if (target->x < 0 || target->x > array.size() - 1 || target->y < 0 || target->y > array[0].size() - 1 ||
        array[target->x][target->y] == 0 || target->x == point->x && target->y == point->y || isInList(closeList, target))
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
            return (array[point->x][target->y] == 0 && array[target->x][point->y] == 0) ? true : isIgnoreCorner;
        }
    }
}

// 寻找父节点周围的点
vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner, vector<vector<int>> objArray)
{
    vector<Point *> surroundPoints;

    for (int x = point->x - 1; x <= point->x + 1; x++)
    {
        for (int y = point->y - 1; y <= point->y + 1; y++)
        {
            if (isCanreach(point, new Point(x, y), isIgnoreCorner, objArray))
            {
                // 加入数组末尾
                surroundPoints.push_back(new Point(x, y));
            }
        }
    }
    return surroundPoints;
}

// 核心代码:寻找最短路径点
Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner, vector<vector<int>> objArray)
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
        auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner, objArray);
        for (auto &target : surroundPoints)
        {
            //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H B
            // 判断完周围点后,如果没有查询过就加入到open中
            if (!isInList(openList, target))
            {
                target->parent = curPoint;

                target->G = calcG(curPoint, target);
                target->H = calcH(target, &endPoint);
                target->B = calcB(target, objArray);
                target->F = calcF(target);

                openList.push_back(target);
            }
            //3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
            // 如果某个相邻方格D已经在 "开启列表" 里了, 检查如果用新的路径 (就是经过C的路径)到达它的话, G值是否会更低一些,如果新的G值更低,那就把它的"父方格"改为目前选中的方格C,
            //      然后重新计算它的 F 值和 G 值 (H 值不需要重新计算, 因为对于每个方块, H 值是不变的). 如果新的 G 值比较高, 就说明经过 C 再到达 D 不是一个明智的选择, 因为它需要更远的路, 这时我们什么也不做.
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
list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner, vector<vector<int>> objArray)
{
    Point *result = findPath(startPoint, endPoint, isIgnoreCorner, objArray);
    list<Point *> path;
    //返回路径，如果没找到路径，返回空链表
    // bool b = nullptr; 正确. if(b)判断为false
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

/*============================================================================*/
/*                                                                            */
/* 地图图片处理模块                                                              */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*============================================================================*/

//  保持图片分辨率，将图像栅格化
//  每次取blk_height*blk_width范围内的RGB值做平均，要注意边界条件
cv::Mat imageChange(cv::Mat image)
{
    using namespace cv;
    // Mat image = imread(filePath);
    imshow("原图", image);

    // int row2 = image.rows;
    // int col2 = image.cols;
    // cout << image.rows << ":" << image.cols << endl;

    int blk_width = 1, blk_height = 2;

    int total_B = 0, total_G = 0, total_R = 0;
    for (int i = 0; i < image.cols; i += blk_width)
    {
        for (int j = 0; j < image.rows; j += blk_height)
        {
            total_B = 0;
            total_G = 0;
            total_R = 0;
            for (int m = 0; m < blk_width; m++)
            {
                // 边界条件
                if (i + m >= image.cols)
                {
                    continue;
                }
                for (int n = 0; n < blk_height; n++)
                {
                    // 累计和
                    // 边界条件
                    if (j + n >= image.rows)
                    {
                        continue;
                    }
                    total_B += image.at<Vec3b>(j + n, i + m)[0];
                    total_G += image.at<Vec3b>(j + n, i + m)[1];
                    total_R += image.at<Vec3b>(j + n, i + m)[2];
                }
            }
            // 均值
            int area = blk_height * blk_width;
            total_B = total_B / area;
            total_G = total_G / area;
            total_R = total_R / area;

            for (int m = 0; m < blk_width; m++)
            {
                // 边界条件
                if (i + m >= image.cols)
                {
                    continue;
                }
                for (int n = 0; n < blk_height; n++)
                {
                    // 边界条件
                    if (j + n >= image.rows)
                    {
                        continue;
                    }
                    image.at<Vec3b>(j + n, i + m)[0] = total_B;
                    image.at<Vec3b>(j + n, i + m)[1] = total_G;
                    image.at<Vec3b>(j + n, i + m)[2] = total_R;
                }
            }
        }
    }
    imshow("栅格图", image);
    waitKey(10000);
    return image;
}

// 改变图片尺寸的方法
cv::Mat colorReduce(cv::Mat &image, int div = 64) //将一张图片的像素改为64x64
{
    using namespace cv;
    //大体思路：将图片中的每个像素的值改为其像素空间中，256/64大小方格中的中间的值。
    // 0.4为是沿x轴和y轴的缩放系数
    resize(image, image, Size(), 0.4, 0.4);
    int n1 = image.rows;                    //图片的行数。
    int nc = image.cols * image.channels(); //列数乘以图片的通道值。因为rgb是逐行连续存储的。r g b
    for (int j = 0; j < n1; j++)
    {
        uchar *data = image.ptr<uchar>(j); //取图片第j行的首个像素点
        for (int i = 0; i < nc; i++)
        {
            data[i] = data[i] / div * div + div / 2; //将此像素值改为其方格中间的像素值。
        }
    }
    imshow("降低分辨率", image);
    return image;
}

// 传入图片,输出二维vecotr和像素点文件，并保存图片到指定地址,flag:(max,min)表示是否缩放搜索图片
cv::Mat imageToVector(string filePath, char num)
{
    using namespace cv;
    // 通过转义字符来消除歧义,imread不需要用这种拼接字符的方式
    // string fileName = "\"" + filePath + "\"";

    Mat img = imread(filePath);
    if (img.empty())
    {
        cout << "图片读取失败" << endl;
    }
    imshow("原图", img);

    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY); //先转为灰度图
    // Mat gray1 = imageChange(gray);
    Mat mapNew;
    Mat gray1;

    char flags = num;
    // 注意 不能在case分支里面定义变量
    switch (flags)
    {
    case 'y':
        gray1 = colorReduce(gray);
        threshold(gray1, mapNew, 127, 255, THRESH_BINARY); //二值化阈值处理
        break;
    case 'n':
        threshold(gray, mapNew, 127, 255, THRESH_BINARY); //二值化阈值处理
        break;
    default:
        break;
    }

    // threshold(gray, mapNew, 127, 255, THRESH_BINARY); //二值化阈值处理
    // threshold(gray1, mapNew, 127, 255, THRESH_BINARY); //二值化阈值处理

    imshow("二值化图", mapNew);

    // 保存像素信息的txt文本路径, 方便查找起点和终点坐标
    fstream file("/home/next/ros_workspace/routing_planning/Astart改进/mapPointsShow.txt", ios::out);
    stringstream ss;
    string data;

    for (int i = 0; i < mapNew.rows; i++)
    {
        for (int j = 0; j < mapNew.cols; j++)
        {
            // maze[i][j] = (mapNew.at<uchar>(i, j) == 255) ? 0 : 1; //img的矩阵数据传给二维数组maze[][]
            // int value = (mapNew.at<uchar>(i, j) == 255) ? 0 : 1;
            int value = mapNew.at<uchar>(i, j);
            ss << dec << value;
            ss >> data;
            ss.clear();
            file << data;
        }
        file << endl;
    }
    cout << "A*地图写入文本成功" << endl;
    waitKey(10000);
    file.close();
    return mapNew;
}

// 给二值图设置膨胀图层
vector<vector<int>> costmap(vector<vector<int>> array, vector<vector<int>> arrayshow, cv::Mat map)
{
    int height = array.size();
    int width = array[0].size();
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            arrayshow[i][j] = map.at<uchar>(i, j); //img的矩阵数据传给二维数组maze[][]
            int value = map.at<uchar>(i, j);
            if (value == 0 && (i - kconstmap) >= 0 && (j - kconstmap) >= 0)
            {
                for (int x = i - kconstmap; x <= i + kconstmap; x++)
                {
                    for (int y = j - kconstmap; y <= j + kconstmap; y++)
                    {
                        // 一定要注意x,y的边界问题
                        if (x < height && y < width)
                        {
                            if (abs(x - i) == kconstmap || abs(y - j) == kconstmap)
                            {
                                array[x][y] = 150;
                            }
                            else
                            {
                                array[x][y] = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    return array;
}
// 为了方便对比膨胀层
vector<vector<int>> costmapshow(vector<vector<int>> array, cv::Mat map)
{
    int height = array.size();
    int width = array[0].size();
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            array[i][j] = map.at<uchar>(i, j); //img的矩阵数据传给二维数组maze[][]
        }
    }
    return array;
}

//传入二维vecotr，显示输出的图片，并保存图片到指定地址
void loadToMap(vector<vector<int>> array)
{
    using namespace cv;
    int h = array.size();
    int w = array[0].size();
    //初始化图片的像素长宽
    //1、2、3表示通道数，如RGB3通道就用CV_8UC3
    // 255为白色，0为黑色
    Mat img = Mat(h, w, CV_8UC1); //如果保存为RGB，图像列数像素要除以3；
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            // if (array[i][j] == 10)
            // {
            //     img.data[i * w + j] = 100;
            // }
            // if (array[i][j] == 4)
            // {
            //     img.data[i * w + j] = 10;
            // }
            // else
            // {
            //     img.data[i * w + j] = 255;
            // }
            img.data[i * w + j] = array[i][j];
        }
    }
    imshow("loadToMap", img);
    waitKey(10000);
    imwrite("/home/next/ros_workspace/routing_planning/Astart改进/loadToMap.jpg", img);
}

/*============================================================================*/
/*                                                                            */
/* 常用函数                                                                    */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*============================================================================*/

// 封装打印数组的方法
void printArray(vector<vector<int>> vec)
{
    vector<int>::iterator it;
    vector<vector<int>>::iterator iter;
    vector<int> vec_tmp;

    for (iter = vec.begin(); iter != vec.end(); iter++)
    {
        vec_tmp = *iter;
        for (it = vec_tmp.begin(); it != vec_tmp.end(); it++)
        {
            cout << *it << " ";
        }
        cout << endl;
    }
}

/*============================================================================*/
/*                                                                            */
/* 平滑路径处理模块                                                              */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*============================================================================*/

// 给生成的路径做平均滤波处理
vector<vector<int>> loadchange(list<Point *> list)
{
    int height = list.size();
    // cout << "list.size(): " << list.size() << endl;
    // 将list中的数传递给数组
    vector<vector<int>> temp(height, vector<int>(2));
    int i = 0;
    for (auto p : list)
    {
        temp[i][0] = p->x;
        temp[i][1] = p->y;
        i++;
    }

    for (int m = 0; m < temp.size(); m++)
    {

        // 注意要初始化赋值
        int xValue = 0, yValue = 0;

        for (int n = (-sampling); n <= sampling; n++)
        {
            if ((m + n) >= 0 && (m + n) < height)
            {
                // xValue = temp[m - 2][0] + temp[m - 1][0] + temp[m][0] + temp[m + 1][0] + temp[m + 2][0];
                xValue += temp[m + n][0];
                // yValue = temp[m - 2][1] + temp[m - 1][1] + temp[m][1] + temp[m + 1][1] + temp[m + 2][1];
                yValue += temp[m + n][1];
            }
        }
        xValue = (int)xValue / (sampling * 2 + 1);
        yValue = (int)yValue / (sampling * 2 + 1);
        temp[m][0] = xValue;
        temp[m][1] = yValue;
    }

    // printArray(temp);
    return temp;
}

int main()
{
    /*============================================================================*/
    cout << "开始处理地图" << endl;
    string filePath = "/home/next/ros_workspace/routing_planning/Astart改进/mapload4.jpg";
    char flags = 'n';
    cv::Mat map = imageToVector(filePath, flags);

    if (!map.empty())
    {
        cout << "A*算法地图初始化和写入" << endl;
    }
    else
    {
        cout << "A*算法地图初始化失败" << endl;
    }

    int width = map.cols;
    int height = map.rows;
    //定义一个矩阵，用于存放数据
    vector<vector<int>> maze(height, vector<int>(width, 255));
    //定义一个矩阵，用于方便查看膨胀过后路径的效果
    vector<vector<int>> mazeshow(height, vector<int>(width, 255));
    cout << "A*算法设置costmap" << endl;
    maze = costmap(maze, mazeshow, map);
    mazeshow = costmapshow(mazeshow, map);
    // printArray(maze);
    /*============================================================================*/

    /*============================================================================*/
    //设置起始和结束点
    Point start(0, 0);
    Point end(390, 390);

    cout << "A*算法开始寻路" << endl;

    clock_t t;   //定义时钟变量t
    t = clock(); //调用前时间

    //A*算法找寻路径
    list<Point *> path = GetPath(start, end, false, maze);
    // cout << "path.size(): " << path.size() << endl;
    if (path.empty())
    {
        cout << "A*算法寻找路径出错,path.size(): " << path.size() << endl;
        return -1;
    }
    //计算函数耗时
    t = clock() - t;
    //转化为秒
    printf("A*算法搜寻路径花费时间=%lf s\n", ((float)t) / CLOCKS_PER_SEC);
    /*============================================================================*/

    /*============================================================================*/
    vector<vector<int>> pathUpdate = loadchange(path);
    // cout << "pathupdate: " << pathUpdate.size() << " " << pathUpdate[0].size() << endl;
    // cout << "mazeshow: " << mazeshow.size() << " " << mazeshow[0].size() << endl;

    // 修改路径点的值,便于观察
    // 未做平滑处理输出
    // for (auto &p : path)
    // {
    //     // cout << '(' << p->x << ',' << p->y << ')' << endl;
    //     maze[p->x][p->y] = 2;
    //     // mazeshow[p->x][p->y] = 8;
    // }
    // 做了平滑处理输出
    for (int m = 0; m < pathUpdate.size(); m++)
    {
        int x = pathUpdate[m][0];
        int y = pathUpdate[m][1];
        mazeshow[x][y] = 8;
    }

    // 使用opencv输出包含路径的图片
    // loadToMap(maze);
    loadToMap(mazeshow);
    /*============================================================================*/

    return 0;
}
