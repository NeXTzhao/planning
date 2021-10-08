/*
 * @Author: your name
 * @Date: 2021-09-29 16:47:44
 * @LastEditTime: 2021-09-29 16:58:44
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /routing_planning/ros/src/littice/src/obstacle.cpp
 */
#include <ros/ros.h>
#include <string>
#include <iostream>


void obstaclePoseCallbcak(const gazebo_msgs::ModelStates& obstacleXYZ) {
  float x = obstacleXYZ.pose.position.x;
  float y = obstacleXYZ.pose.position.y;
  cout << "x=" << x << "; y=" << y << endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle");
  ros::NodeHandle ob;
  ros::Subscriber obstacle =
      ob.subscribe("/gazebo/model_states", 20, obstaclePoseCallbcak);
  ros::spin();
  return 0;
}