/***
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-04-05 11:37:39
 * @LastEditors: your name
 * @Brief:
 * @FilePath: /ROS2_WORKSPACE/src/myint/src/myPublish.cpp
 * @Copyright:
 */
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "myint/msg/num.hpp"

using namespace std::chrono_literals;

class miniPub : public rclcpp::Node {
 public:
  miniPub() : Node("minimal_publish"), count_(0) {
    publisher_ = this->create_publisher<myint::msg::Num>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&miniPub::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = myint::msg::Num();
    message.num = this->count_++;
    RCLCPP_INFO(this->get_logger(), "publish: '%d'", message.num);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<myint::msg::Num>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<miniPub>());
  return 0;
}