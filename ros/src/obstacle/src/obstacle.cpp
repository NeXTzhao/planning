/*
 * @Author: your name
 * @Date: 2021-09-29 16:47:44
 * @LastEditTime: 2021-09-29 17:45:23
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /routing_planning/ros/src/littice/src/obstacle.cpp
 */
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <iostream>
#include <string>

using namespace std;

void obstaclePoseCallbcak(const gazebo_msgs::ModelStates& obstacleXYZ) {
  int obsNum = obstacleXYZ.pose.size();
  for (int i = 0; i < obsNum; i++) {
    float x = obstacleXYZ.pose[i].position.x;
    float y = obstacleXYZ.pose[i].position.y;
    cout << "x=" << x << "; y=" << y << endl;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle");
  ros::NodeHandle ob;
  ros::Subscriber obstacle =
      ob.subscribe("/gazebo/model_states", 20, obstaclePoseCallbcak);
  ros::spin();
  return 0;
}