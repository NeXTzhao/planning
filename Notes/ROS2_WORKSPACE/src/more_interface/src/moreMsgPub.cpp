/*
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-04-05 16:08:37
 * @LastEditors: your name
 * @Brief:
 * @FilePath: /ROS2_WORKSPACE/src/more_interface/src/moreMsgPub.CPP
 * @Copyright:
 */
#include <chrono>
#include <memory>

#include "more_interface/msg/address_book.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class moreMsgPub : public rclcpp::Node {
 public:
  moreMsgPub() : Node("address_book_publish") {
    address_book_ = this->create_publisher<more_interface::msg::AddressBook>(
        "address_book", 10);
    auto publish_msg = [this]() -> void {
      auto message = more_interface::msg::AddressBook();

      message.first_name = "John";
      message.last_name = "Doe";
      message.age = 30;
      message.gender = message.MALE;
      message.address = "unknown";

      std::cout << "Publishing Contact\nFirst:" << message.first_name
                << "  Last:" << message.last_name << std::endl;

      this->address_book_->publish(message);
    };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

 private:
  rclcpp::Publisher<more_interface::msg::AddressBook>::SharedPtr address_book_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<moreMsgPub>());
  rclcpp::shutdown();

  return 0;
}