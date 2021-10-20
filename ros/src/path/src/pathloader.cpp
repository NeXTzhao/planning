/*
 * @Author: your name
 * @Date: 2021-09-18 19:38:06
 * @LastEditTime: 2021-09-19 14:43:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ros/src/pathloader.cpp
 */

#include <sstream>
#include <ros/ros.h>
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    // 初始化
    ros::init(argc, argv, "path_ros");

    // 创建句柄
    ros::NodeHandle n;

    // 创建一个publish,发布名为path的topic ,消息类型为stc_msgs::String
    ros::Publisher patter = n.advertise<std_msgs::String>("path", 1000);

    // 设置循环的频率`
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello" << count;
        msg.data = ss.str();

        //发布消息
        ROS_INFO("%s", msg.data.c_str());//打印消息
        patter.publish(msg);

        //循环等待回调函数
        ros::spinOnce();

        //按照循环频率延时
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
