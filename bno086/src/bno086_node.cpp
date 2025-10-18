/**
 * @file bno086_node.cpp
 * @author Yamaguchi Yudai
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <chrono>

#include "bno086/bno086_driver.h"
#include "bno086/serial_driver.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("bno086_node")
  {
    serial_ = std::make_shared<SerialDriver>("/dev/ttyUSB0");
    bno086_driver_ = std::make_shared<BNO086Driver>(serial_);
    timer_publisher_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));
  }

  void timer_callback(void)
  {
    bno086_driver_->update();
    bno086_driver_->print(bno086_driver_->getData());
  }

  std::shared_ptr<SerialDriver> serial_;
  std::shared_ptr<BNO086Driver> bno086_driver_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}