#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
// using namespace cv;

/**
 保持图片分辨率，将图像栅格化
 每次取blk_height*blk_width范围内的RGB值做平均，要注意边界条件
 */
void imageChange(string filePath)
{	using namespace cv;
	cv::Mat image = cv::imread(filePath);
	imshow("ori", image);

	// cout << image.rows << ":" << image.cols << endl;

	// int row2 = image.rows;
	// int col2 = image.cols;

	int blk_width = 3, blk_height = 3;

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
	cv::imshow("image", image);
	cv::waitKey(10000);
}

int main()
{
	// Mat image = imread("/home/next/ros_workspace/map_file/AStart/mapload.jpg");
	// resize(image, image, Size(), 0.4, 0.4);
	// colorReduce5(image, 64);

	// imshow("结果", image);

	// waitKey(0);

	
	imageChange("/home/next/ros_workspace/map_file/mapload1.jpg");

	return 0;
}
