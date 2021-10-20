/*
 * @Author: your name
 * @Date: 2021-09-20 10:47:51
 * @LastEditTime: 2021-09-20 12:09:31
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ros/test/test.cpp
 */
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

/**
 * @brief 将cvs文件写入二位数组中，其中如果csv以逗号为分隔符
 *  如果是以空格号为分隔符 getline(sin, field, ' ');把逗号改成空格号就行
 * 
 * @return int 
 */

vector<vector<float>> csvToVector(string filepath)
{
    ifstream inFile(filepath, ios::in);
    if (!inFile)
    {
        cout << "打开文件失败！" << endl;
        exit(1);
    }
    int i = 0;
    string line, field;

    int height = 12605, weight = 3;
    vector<vector<float>> pathpoints(height, vector<float>(weight));

    while (getline(inFile, line)) //getline(inFile, line)表示按行读取CSV文件中的数据
    {
        istringstream sin(line); //将整行字符串line读入到字符串流sin中

        getline(sin, field, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
        // cout << atof(field.c_str()) << " " << endl; //将刚刚读取的字符串转换成int
        // cout << "file1:" << field << endl;
        // int a = field.size();
        // printf("\n a= %d  is\n", a);
        pathpoints[i][0] = atof(field.c_str());

        getline(sin, field, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
        // cout << atof(field.c_str()) << " " << endl; //将刚刚读取的字符串转换成int
        // cout << "file2:" << field << endl;
        // int b = field.size();
        // printf("\nb %d is \n", b);
        pathpoints[i][1] = atof(field.c_str());

        getline(sin, field); //将字符串流sin中的字符读入到field字符串中，读完为止
        // cout << atof(field.c_str()) << endl; //将刚刚读取的字符串转换成int
        // cout << "file3:" << field << endl;
        pathpoints[i][2] = atof(field.c_str());

        i++;
    }
    inFile.close();
    // cout << "共读取了：" << i << "行" << endl;
    // cout << "读取数据完成" << endl;

    return pathpoints;
}
int main()
{
    string filepath = "/home/next/ros_workspace/routing_planning/数据处理脚本/waypoints.csv";

    auto pathpoints = csvToVector(filepath);

    cout << pathpoints[0][0] << "," << pathpoints[0][1] << "," << pathpoints[0][2] << endl;

    return 0;
}