#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>

#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "modules/visualization/visualize.h"

std::shared_ptr<Visualize> vis;

void cb_hdmap(const msgs::SMAO_HDMapMsg_st &msg) {
  ROS_INFO("Receive hdmap");
  vis->updateHDmap(msg);
}

void cb_position(const geometry_msgs::PoseStamped &msg) {
  ROS_INFO("Receive position");
  vis->updatePosition(msg);
}

void cb_prediction(const msgs::PredictionObstacles &msg) {
  ROS_INFO("Receive prediction");
  vis->updatePrediction(msg);
}

void cb_reference_line(const std_msgs::Float32MultiArray &msg) {
  ROS_INFO("Receive reference line");
  vis->updateReferenceLine(msg);
}

void cb_trajectory(const std_msgs::Float32MultiArray &msg) {
  ROS_INFO("Receive trajectory");
  vis->updateTrajectory(msg);
}
void cb_path(const nav_msgs::Path &msg) {
  ROS_INFO("Receive record path");
  vis->updatePath(msg);
}
void cb_speed(const msgs::WheelSpeedReport &msg) {
  ROS_INFO("Receive wheel speed");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pla_vis_node");
  ros::NodeHandle n;

  vis = std::make_shared<Visualize>();

  ros::Subscriber sub_HDmap = n.subscribe("/SMAO_HDMapMsg_st", 10, cb_hdmap);
  ros::Subscriber sub_Position =
      n.subscribe("/hpa_location/fusion_pose", 10, cb_position);
  ros::Subscriber sub_Prediction =
      n.subscribe("/prediction", 10, cb_prediction);
  ros::Subscriber sub_RefrenceLine =
      n.subscribe("/reference_line", 1000, cb_reference_line);
  ros::Subscriber sub_Trajectory =
      n.subscribe("/Trajectory_Debug", 1000, cb_trajectory);
  ros::Subscriber sub_Path =
      n.subscribe("/hpa_location/map_line_path", 1000, cb_path);
  ros::Subscriber sub_Speed =
      n.subscribe("/vehicle/WheelSpeedReport", 1000, cb_speed);

  double sim_step = 0.1;
  ros::Rate rate(1 / sim_step);
  while (1) {
    // 1.get input message
    ros::spinOnce();
    vis->pubData();
    rate.sleep();
  }
  return 0;
}