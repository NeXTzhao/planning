/***
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-04-05 12:47:09
 * @LastEditors: your name
 * @Brief:
 * @FilePath: /ROS2_WORKSPACE/src/my_package/src/mySub.cpp
 * @Copyright:
 */
#include <memory>

#include "myint/msg/num.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class miniSub : public rclcpp::Node {
 public:
  miniSub() : Node("miniSub") {
    subscription_ = this->create_subscription<myint::msg::Num>(
        "topic", 10, std::bind(&miniSub::topic_callback, this, _1));
  }

 private:
  void topic_callback(const myint::msg::Num::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d", msg->num);
  }
  rclcpp::Subscription<myint::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<miniSub>());
  rclcpp::shutdown();
  return 0;
}