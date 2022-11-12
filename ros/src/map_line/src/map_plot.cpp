#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <vector>

int pointNum = 0;
std::vector<double> r_x_, r_y_;

void map_parse(const nav_msgs::Path &msg) {
  // geometry_msgs/PoseStamped[] poses
  pointNum = msg.poses.size();

  // auto a = msg.poses[0].pose.position.x;
  for (int i = 0; i < pointNum; i++) {
    r_x_.push_back(msg.poses[i].pose.position.x);
    r_y_.push_back(msg.poses[i].pose.position.y);
    printf("x=%f,y=%f\n", msg.poses[i].pose.position.x,
           msg.poses[i].pose.position.y);
  }
}

void test() {
  for (int i = 0; i < pointNum; ++i) {
    printf("x=%f,y=%f\n", r_x_.at(i), r_y_.at(i));
  }
}

int main(int argc, char **argv) {
  //创建节点
  printf("hello world\n");
  ros::init(argc, argv, "map");
  // 创建节点句柄
  ros::NodeHandle n;

  ros::Subscriber rosbagmessage =
      n.subscribe("/hpa_location/map_line_path", 20, map_parse);
  // test();

  ros::spin();

  return 0;
}
