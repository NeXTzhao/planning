#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cpprobotics_types.h"
#include "frenet_path.h"
#include "quintic_polynomial.h"

using namespace std;

#define DT 0.1    // time tick [s]
#define MAXT 5.0  // max prediction time [m]
#define MINT 4.0  // min prediction time [m]

using namespace cpprobotics;

// t-t0经历的时间
float T = 10;

float xend = 50.0;
float yend = 25.0;

// 起始状态
std::array<float, 3> x_start{0.0, 0.0, 0.0};
std::array<float, 3> x_end{xend, 0.0, 0.0};
// 终点状态
std::array<float, 3> y_start{0.0, 0.0, 0.0};
std::array<float, 3> y_end{yend, 0.0, 0.0};

FrenetPath calc_frenet_paths() {
  FrenetPath fp;
  // 纵向
  QuinticPolynomial lon_qp(x_start[0], x_start[1], x_start[2], x_end[0],
                           x_end[1], x_end[2], T);
  // 横向
  QuinticPolynomial lat_qp(y_start[0], y_start[1], y_start[2], y_end[0],
                           y_end[1], y_end[2], T, xend);

  for (float t = 0; t < 10; t += DT) {
    float x = lon_qp.calc_point_x(t);
    float xd = lon_qp.calc_point_xd(t);
    float xdd = lon_qp.calc_point_xdd(t);
    fp.t.push_back(t);
    fp.x.push_back(x);
    fp.x_d.push_back(xd);
    fp.x_dd.push_back(xdd);

    float y_x_t = lat_qp.calc_point_y_x(x);
    float y_x_d = lat_qp.calc_point_y_x_d(x);
    float y_x_t_d = lat_qp.calc_point_y_t_d(y_x_d, xd);

    float y_x_dd = lat_qp.calc_point_y_x_dd(x);
    float y_x_t_dd = lat_qp.calc_point_y_t_dd(y_x_dd, xd, y_x_d, xdd);

    fp.y.push_back(y_x_t);
    fp.y_d.push_back(y_x_t_d);
    fp.y_dd.push_back(y_x_t_dd);
    // 压入航向角
    // fp.threat.push_back(lat_qp.calc_point_thetar(y_x_t));
    fp.threat.push_back(lat_qp.calc_point_thetar(y_x_t_d, xd));

    // 压入曲率
    fp.k.push_back(lat_qp.calc_point_k(y_x_dd, y_x_d));
    // fp.k.push_back(lat_qp.calc_point_k(y_x_t_dd, y_x_t_d, xdd, xd));
  }

  return fp;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "quinticpath");

  ros::NodeHandle f;
  ros::Publisher path_pub =
      f.advertise<nav_msgs::Path>("quinticpathpoints", 1000, true);
  // ros::Rate loop_rate(10);
  nav_msgs::Path now_path;

  now_path.header.frame_id = "world";
  // 设置时间戳
  now_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  // 设置参考系
  pose.header.frame_id = "world";

  FrenetPath fp = calc_frenet_paths();

  int sNum = fp.x.size();
  for (int i = 0; i < sNum; i++) {
    printf("x,y,th,k=%.3f,%.3f,%.3f,%.3f\n", fp.x[i], fp.y[i], fp.threat[i],
           fp.k[i]);
  }
  while (ros::ok()) {
    for (int i = 0; i < sNum; i++) {
      pose.pose.position.x = fp.x[i];
      pose.pose.position.y = fp.y[i];
      pose.pose.position.z = 0;

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 0.0;
      //   pose.pose.orientation.w = 0.0;
      now_path.poses.push_back(pose);
      // ROS_INFO("POSE WRITE IS OK!");
    }
    path_pub.publish(now_path);
    ros::spin();
    // loop_rate.sleep();
  }

  return 0;
}
