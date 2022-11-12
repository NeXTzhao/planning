/***
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-04-04 19:25:34
 * @LastEditors: your name
 * @Brief:
 * @FilePath: /Notes/ROS2/Subscriber.cpp
 * @Copyright:
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}