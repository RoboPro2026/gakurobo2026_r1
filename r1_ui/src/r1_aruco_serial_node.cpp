/**
 * @file r1_aruco_serial_node.cpp
 * @author Yudai Yamaguchi (yudai.yy0804@gmail.com)
 * @brief シリアル通信で小型ディスプレイにarucoマーカを表示するノード
 * @version 0.1
 * @date 2026-04-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "r1_ui/serial_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class R1ArucoSerialNode : public rclcpp::Node
{
public:
  R1ArucoSerialNode() : Node("r1_aruco_serial_node")
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

    aruco_marker_id_ = this->create_subscription<std_msgs::msg::Int32>(
      "aruco_marker_id", 10,
      std::bind(&R1ArucoSerialNode::aruco_marker_id_callback, this, std::placeholders::_1));
    // 10Hz
    timer_ = this->create_wall_timer(100ms, std::bind(&R1ArucoSerialNode::timer_callback, this));
  }

  void timer_callback()
  {
    std::vector<uint8_t> rx_buff = serial_->read();
    for (int i = 0; i < (int)rx_buff.size(); i++) {
      recv_buff_[recv_index_++] = rx_buff[i];
      if (rx_buff[i] == '\0') {
        RCLCPP_INFO(this->get_logger(), "%s", recv_buff_);
        recv_index_ = 0;  // インデックスをリセット
      }
    }
  }

  void aruco_marker_id_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int marker_id = msg->data;
    // 改行文字は送信しない
    std::string send_str = std::to_string(marker_id);
    std::vector<uint8_t> tx_buff(send_str.begin(), send_str.end());
    serial_->write_buff(tx_buff);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_marker_id_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<SerialDriver> serial_;
  char recv_buff_[256];
  int recv_index_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1ArucoSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
