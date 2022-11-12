#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
using namespace std;

int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("/home/next/routing_planning/pcd/row.pcd", *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix4f rotation_x =
      Eigen::Matrix4f::Identity();  //定义绕X轴的旋转矩阵，并初始化为单位阵
  double angle_x = M_PI / 2;  //旋转90°
  rotation_x(1, 1) = cos(angle_x);
  rotation_x(1, 2) = -sin(angle_x);
  rotation_x(2, 1) = sin(angle_x);
  rotation_x(2, 2) = cos(angle_x);
  pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_x);

  pcl::io::savePCDFileASCII("row_conv.pcd",
                            *cloud_transformed);  //将点云保存到PCD文件中

  return 0;
}