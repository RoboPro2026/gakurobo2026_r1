/**
 * @file bno086_node.cpp
 * @author Yamaguchi Yudai
 * @brief bno086のROS 2ノード
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// TODO: tf2を用いた座標変換を実装する

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
    // パラメータを宣言
    this->declare_parameter<std::string>("port");
    // パラメータを取得
    std::string port_name = this->get_parameter("port").as_string();

    RCLCPP_INFO(this->get_logger(), "Connecting to port: %s", port_name.c_str());

    // 取得したパラメータで初期化
    try {
      serial_ = std::make_shared<SerialDriver>(port_name);
      if (serial_->get_is_initialize_success() == false) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open port: %s.", port_name.c_str());
        rclcpp::shutdown();
        return;  // コンストラクタが終了すれば main も終了する
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to open port: %s. Error: %s", port_name.c_str(), e.what());
      // rclcpp::shutdown() を呼ぶか、例外を投げて終了させる
      rclcpp::shutdown();
      return;  // コンストラクタが終了すれば main も終了する
    }

    bno086_driver_ = std::make_shared<BNO086Driver>(serial_);
    timer_publisher_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));
  }

  void timer_callback(void)
  {
    bno086_driver_->update();
    bno086_driver_->print(bno086_driver_->get_data());
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