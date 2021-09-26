/*
 * @Author: your name
 * @Date: 2021-09-19 19:51:13
 * @LastEditTime: 2021-09-26 14:46:32
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ros/src/pathplay/src/pathload.cpp
 */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "cpprobotics_types.h"
#include "cubic_spline.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spline");

    ros::NodeHandle sp;
    ros::Publisher path_pub = sp.advertise<nav_msgs::Path>("splinepoints", 1000, true);

    // ros::Rate loop_rate(10);
    // ros::Rate loop_rate(1);
    int count = 0;
    nav_msgs::Path now_path;

    now_path.header.frame_id = "world";
    // 设置时间戳
    now_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    // 设置参考系
    pose.header.frame_id = "world";

    cpprobotics::Vec_f wx({0.0, 10.0, 20.5, 35.0, 70.5});
    cpprobotics::Vec_f wy({0.0, -6.0, 5.0, 6.5, 0.0});

    cpprobotics::Spline2D csp_obj(wx, wy);
    for (float i = 0; i < csp_obj.s.back(); i += 0.1)
    {
        std::array<float, 2> point_ = csp_obj.calc_postion(i);

        pose.pose.position.x = point_[0];
        pose.pose.position.y = point_[1];
        pose.pose.position.z = 0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        now_path.poses.push_back(pose);
        // ROS_INFO("POSE WRITE IS OK!");
    }

    path_pub.publish(now_path);

    ros::spin();
    // loop_rate.sleep();

    return 0;
}
