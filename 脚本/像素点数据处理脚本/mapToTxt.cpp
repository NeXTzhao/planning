#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

/*
将图片二值化再写入到文本
*/

int main()
{
    string filename = "/home/next/ros_workspace/map_file/AStart/loadToMap1.jpg";
    Mat img = imread(filename);
    if (img.empty())
    {
        cout << "NULL";
        return 0;
    }
    int row = img.rows;
    int col = img.cols;
    namedWindow("原图");
    imshow("原图", img);

    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY); //先转为灰度图

    Mat mapNew;
    threshold(gray, mapNew, 127, 255, THRESH_BINARY); //二值化阈值处理
    imshow("gray_image", mapNew);

    // 保存像素信息的txt文本路径
    // fstream file("/home/next/ros_workspace/map_file/AStart/loadTomap1.txt", ios::out);
    // stringstream ss;
    // string data0;

    fstream file1("/home/next/ros_workspace/map_file/AStart/loadTomap1.txt", ios::out);
    stringstream ss1;
    string data1;

    // for (int i = 0; i < row; i++)
    // {
    //     for (int j = 0; j < col; j++)
    //     {
    //         // ptr[i][j] = mapNew.at<uchar>(i, j);//img的矩阵数据传给二维数组ptr[][]
    //         int value = (mapNew.at<uchar>(i, j) == 255) ? 0 : 1;

    //         ss << dec << value;
    //         ss >> data0;
    //         ss.clear();
    //         file << data0 << " ";
    //     }
    //     file << endl;
    // }

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            // ptr[i][j] = mapNew.at<uchar>(i, j);//img的矩阵数据传给二维数组ptr[][]
            int value = (mapNew.at<uchar>(i, j) == 255) ? 0 : 1;

            ss1 << dec << value;
            ss1 >> data1;
            ss1.clear();
            file1 << data1;
        }
        file1 << endl;
    }

    waitKey(100000);
    return 0;
}