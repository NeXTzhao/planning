/*
 * @Author: your name
 * @Date: 2021-08-31 14:59:41
 * @LastEditTime: 2021-09-20 12:01:24
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /ros/home/next/ros_workspace/routing_planning/数据处理脚本/pointsToArray.cpp
 */
#include <iostream>
#include <fstream>
#include <string>
#include <typeinfo>
#include <vector>

using namespace std;

#define MAX_NODE 500000
#define row 720
#define low 1281

/**
 * @brief 将csv,txt写入数组的另一种方法
 * 
 * @return int 
 */
int main()
{
    vector<vector<int>> vecLoad(row, vector<int>(low));

    int data[720][1281] = {0};    //定义一个矩阵，用于存放数据
    ifstream infile;              //定义读取文件流
    infile.open("mapPoints.txt"); //打开文件
    for (int i = 0; i < 720; i++) //定义行循环
    {
        for (int j = 0; j < 1281; j++) //定义列循环
        {
            // infile >> data[i][j];
            infile >> vecLoad[i][j]; //读取一个值（空格、制表符、换行隔开）就写入到矩阵中，行列不断循环进行
        }
    }
    infile.close(); //读取完成之后关闭文件
    // cout << data[3][0] << ',' << data[3][1] << endl; //以下代码是用来验证读取到的值是否正确
    // cout << data[234][897] << ',' << data[234][898] << endl;
    // cout << typeid(data[3][0]).name() << endl;

    cout << vecLoad[3][0] << ',' << vecLoad[3][1] << endl; //以下代码是用来验证读取到的值是否正确
    cout << vecLoad[234][897] << ',' << vecLoad[234][898] << endl;
    cout << typeid(vecLoad[3][0]).name() << endl;

    // 打印数组
    for (int i = 0; i < vecLoad.size(); i++) //输出二维动态数组
    {
        for (int j = 0; j < vecLoad[i].size(); j++)
        {
            cout << vecLoad[i][j];
        }
        cout << "\n";
    }

    cout << "写入成功" << endl;
    return 0;
}
