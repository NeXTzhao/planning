#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <string>

// int main(int argc, char** argv) {
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
//   pcl::PointCloud<pcl::PointXYZ>); pcl::PointCloud<pcl::PointXYZ>::Ptr
//   cloud1(
//       new pcl::PointCloud<pcl::PointXYZ>);

//   //*cloud,指针的内容是文件内容，记得标明点云类型<pcl::PointXYZ>
//   if (pcl::io::loadPCDFile<pcl::PointXYZ>(
//           "/home/next/routing_planning/pcd/row.pcd", *cloud) == -1) {
//     PCL_ERROR(
//         "Couldn't read file test_pcd.pcd\n");  //
//         pcl有专门的报错函数PCL_ERROR
//     return (-1);
//   }
//   if (pcl::io::loadPCDFile<pcl::PointXYZ>(
//           "/home/next/routing_planning/pcd/lidar20220919.pcd", *cloud1) ==
//           -1) {
//     PCL_ERROR(
//         "Couldn't read file test_pcd.pcd\n");  //
//         pcl有专门的报错函数PCL_ERROR
//     return (-1);
//   }
//   pcl::visualization::CloudViewer viewer(
//       "pcd viewer");        //给显示窗口命名，CloudViewer
//   viewer.showCloud(cloud);  //定义要显示的对象,showCloud

//   pcl::visualization::CloudViewer viewer1(
//       "pcd viewer1");         //给显示窗口命名，CloudViewer
//   viewer1.showCloud(cloud1);  //定义要显示的对象,showCloud

//   while (!viewer.wasStopped()) {
//   };
//   while (!viewer1.wasStopped()) {
//   };
//   system("pause");  //此处防止显示闪退
//   return (0);
// }

using namespace pcl;
using namespace pcl::io;
using namespace std;

int main() {
  //显示原始点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/home/next/routing_planning/pcd/build/row_conv.pcd", *cloud) != -1) {
    pcl::visualization::CloudViewer viewer("original pointcloud");
    viewer.showCloud(cloud);
    std::cout << "original pointcloud:" << cloud->points.size() << std::endl;
    while (!viewer.wasStopped()) {
    }
  } else {
    PCL_ERROR("Couldnot read file.\n");
  }

  //显示转换
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(
  //     new pcl::PointCloud<pcl::PointXYZ>);
  // if (pcl::io::loadPCDFile<pcl::PointXYZ>(
  //         "/home/next/routing_planning/pcd/row.pcd", *cloud2) != -1) {
  //   pcl::visualization::CloudViewer viewer2("nofloor_cloud");
  //   viewer2.showCloud(cloud2);
  //   std::cout << "no floor cloud numbers:" << cloud2->points.size()
  //             << std::endl;
  //   while (!viewer2.wasStopped()) {
  //   }

  // } else {
  //   PCL_ERROR("Couldnot read file.\n");
  // }

  cout << "按任意键继续……";

  //下面的代码是为了解决在运行中遇到的一闪而过的问题
  cin.clear();
  cin.sync();
  cin.get();
  return 0;
}