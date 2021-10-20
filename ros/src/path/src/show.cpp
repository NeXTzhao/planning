/*
 * @Author: your name
 * @Date: 2021-09-18 20:31:46
 * @LastEditTime: 2021-09-18 21:16:10
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ros/src/path/src/show.cpp
 */

#include <ros/ros.h>
#include "std_msgs/String.h"

void show(const std_msgs::String::ConstPtr &msg)
{
    //接收的信息打印
    ROS_INFO("%s",msg->data.c_str());

}

int main(int argv ,char **args)
{
    ros::init(argv,args,"show");
    ros::NodeHandle w;
    //注册show回调函数
    ros::Subscriber sub = w.subscribe("path",1000,show);
    ros::spin();

    return 0;
}
