#include <iostream>
#include <fstream>
#include <string>
#include <typeinfo>
#include <vector>
#include "AStar.h"
using namespace std;

#define row 720
#define low 1281

int main()
{
	//初始化地图，用二维矩阵代表地图，1表示障碍物，0表示可通
	// vector< vector<int> > vec(row, low);
	// vector< vector<int> > maze = {
	// 	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	// 	{1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1},
	// 	{1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1},
	// 	{1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1},
	// 	{1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
	// 	{1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
	// 	{1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
	// 	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

	vector< vector<int> > vecLoad(row, vector<int>(low));

	int data[720][1281] = {0};											   //定义一个矩阵，用于存放数据
	ifstream infile;													   //定义读取文件流
	infile.open("/home/next/ros_workspace/map_file/AStart/mapPoints.txt"); //打开文件
	for (int i = 0; i < 720; i++)										   //定义行循环
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

	// cout << vecLoad[3][0] << ',' << vecLoad[3][1] << endl; //以下代码是用来验证读取到的值是否正确
	// cout << vecLoad[234][897] << ',' << vecLoad[234][898] << endl;
	// cout << typeid(vecLoad[3][0]).name() << endl;

	// 打印数组
	// for (int i = 0; i < vecLoad.size(); i++) //输出二维动态数组
	// {
	// 	for (int j = 0; j < vecLoad[i].size(); j++)
	// 	{
	// 		cout << vecLoad[i][j];
	// 	}
	// 	cout << "\n";
	// }

	// cout << "Hello" << endl;
	cout << "A*算法地图导入成功" << endl;
	Astar astar;
	astar.InitAstar(vecLoad);

	//设置起始和结束点
	Point start(66, 242);
	Point end(534, 183);
	cout << "A*算法开始寻找路径" << endl;
	//A*算法找寻路径
	list<Point *> path = astar.GetPath(start, end, false);
	cout << "A*算法寻路成功" << endl;
	//打印
	for (auto &p : path)
	{
		cout << '(' << p->x << ',' << p->y << ')' << endl;
	}
	return 0;
}