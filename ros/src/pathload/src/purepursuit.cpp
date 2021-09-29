// #include <math.h>
#include <ros/ros.h>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

#include "cpprobotics_types.h"
#include "cubic_spline.h"

#define PREVIEW_DIS 4.8  //预瞄距离

#define Ld 1.868  //轴距

using namespace std;
using namespace cpprobotics;

ros::Publisher purepersuit;
ros::Publisher path_pub;
nav_msgs::Path path;

float carVelocity = 0;
float preview_dis = 0;
float k = 0.1;

// 计算四元数转换到欧拉角
std::array<float, 3> calQuaternionToEuler(const float x, const float y,
                                          const float z, const float w) {
  std::array<float, 3> calRPY = {(0, 0, 0)};
  // roll = atan2(2(wx+yz),1-2(x*x+y*y))
  calRPY[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  // pitch = arcsin(2(wy-zx))
  calRPY[1] = asin(2 * (w * y - z * x));
  // yaw = atan2(2(wx+yz),1-2(y*y+z*z))
  calRPY[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

  return calRPY;
}

cpprobotics::Vec_f r_x_;
cpprobotics::Vec_f r_y_;

int pointNum = 0;  //保存路径点的个数
int targetIndex = pointNum - 1;
vector<int> bestPoints_ = {pointNum - 1};
//计算发送给模型车的转角
void poseCallback(const geometry_msgs::PoseStamped &currentWaypoint) {
  auto currentPositionX = currentWaypoint.pose.position.x;
  auto currentPositionY = currentWaypoint.pose.position.y;
  auto currentPositionZ = 0.0;

  auto currentQuaternionX = currentWaypoint.pose.orientation.x;
  auto currentQuaternionY = currentWaypoint.pose.orientation.y;
  auto currentQuaternionZ = currentWaypoint.pose.orientation.z;
  auto currentQuaternionW = currentWaypoint.pose.orientation.w;

  std::array<float, 3> calRPY =
      calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
                           currentQuaternionZ, currentQuaternionW);
  // 寻找匹配目标点
  for (int i = 0; i < pointNum; i++) {
    float lad = 0.0;  //累加前视距离

    float next_x = r_x_[i + 1];
    float next_y = r_y_[i + 1];
    lad += sqrt(pow(next_x - currentPositionX, 2) +
                pow(next_y - currentPositionY, 2));
    // cos(aAngle)判断方向
    float aAngle =
        atan2(next_y - currentPositionY, next_x - currentPositionX) - calRPY[2];
    if (lad > preview_dis && cos(aAngle) >= 0) {
      targetIndex = i + 1;
      bestPoints_.push_back(targetIndex);
      break;
    }
  }

  // 取容器中的最大值
  int index = *max_element(bestPoints_.begin(), bestPoints_.end());
  // cout << "index:" << index << endl;
  float alpha =
      atan2(r_y_[index] - currentPositionY, r_x_[index] - currentPositionX) -
      calRPY[2];

  // 当前点和目标点的距离Id
  float dl = sqrt(pow(r_y_[index] - currentPositionY, 2) +
                  pow(r_x_[index] - currentPositionX, 2));

  if (dl > 0.2) {
    float theta = atan(2 * Ld * sin(alpha) / dl);
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 3;
    vel_msg.angular.z = theta;
    purepersuit.publish(vel_msg);
    // 发布小车运动轨迹
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = currentPositionX;
    this_pose_stamped.pose.position.y = currentPositionY;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(theta);
    this_pose_stamped.pose.orientation.x = currentQuaternionX;
    this_pose_stamped.pose.orientation.y = currentQuaternionY;
    this_pose_stamped.pose.orientation.z = currentQuaternionZ;
    this_pose_stamped.pose.orientation.w = currentQuaternionW;

    this_pose_stamped.header.stamp = ros::Time::now();

    this_pose_stamped.header.frame_id = "world";
    path.poses.push_back(this_pose_stamped);
  } else {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    purepersuit.publish(vel_msg);
  }
  path_pub.publish(path);
}

void velocityCall(const geometry_msgs::TwistStamped &carWaypoint) {
  // carVelocity = carWaypoint.linear.x;
  carVelocity = carWaypoint.twist.linear.x;
  preview_dis = k * carVelocity + PREVIEW_DIS;
}

void pointCallback(const nav_msgs::Path &msg) {
  // geometry_msgs/PoseStamped[] poses
  pointNum = msg.poses.size();

  // auto a = msg.poses[0].pose.position.x;
  for (int i = 0; i < pointNum; i++) {
    r_x_.push_back(msg.poses[i].pose.position.x);
    r_y_.push_back(msg.poses[i].pose.position.y);
  }
}

int main(int argc, char **argv) {
  //创建节点
  ros::init(argc, argv, "pure_pursuit");

  //创建节点句柄
  ros::NodeHandle n;
  //创建Publisher，发送经过pure_pursuit计算后的转角及速度
  purepersuit = n.advertise<geometry_msgs::Twist>("/smart/cmd_vel", 20);

  path_pub = n.advertise<nav_msgs::Path>("rvizpath", 100, true);
  // ros::Rate loop_rate(10);

  path.header.frame_id = "world";
  // 设置时间戳
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  // 设置参考系
  pose.header.frame_id = "world";

  ros::Subscriber splinePath = n.subscribe("/splinepoints", 20, pointCallback);
  ros::Subscriber carVel = n.subscribe("/smart/velocity", 20, velocityCall);
  ros::Subscriber carPose = n.subscribe("/smart/rear_pose", 20, poseCallback);
  ros::spin();
  return 0;
}
