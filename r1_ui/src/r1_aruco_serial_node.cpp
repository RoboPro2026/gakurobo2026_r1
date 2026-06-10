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

#include <optional>

#include "r1_ui/serial_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class R1ArucoSerialNode : public rclcpp::Node
{
public:
  R1ArucoSerialNode() : Node("r1_aruco_serial_node")
  {
    this->declare_parameter<std::string>("port");
    this->declare_parameter<double>("timer_rate", timer_rate_);
    this->declare_parameter<double>("reconnect_interval_sec", reconnect_interval_sec_);

    port_name_ = this->get_parameter("port").as_string();
    timer_rate_ = this->get_parameter("timer_rate").as_double();
    reconnect_interval_sec_ = this->get_parameter("reconnect_interval_sec").as_double();

    if (timer_rate_ <= 0) {
      RCLCPP_WARN(
        this->get_logger(), "Invalid timer_rate: %f. Using default value: %f", timer_rate_, 10.0);
      timer_rate_ = 10.0;
    }

    RCLCPP_INFO(this->get_logger(), "Connecting to port: %s", port_name_.c_str());

    try {
      serial_ = std::make_shared<SerialDriver>(port_name_);
      if (!serial_->is_connected()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open port: %s.", port_name_.c_str());
        rclcpp::shutdown();
        return;
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to open port: %s. Error: %s", port_name_.c_str(), e.what());
      rclcpp::shutdown();
      return;
    }

    aruco_marker_id_ = this->create_subscription<std_msgs::msg::Int32>(
      "aruco_marker_id", 10,
      std::bind(&R1ArucoSerialNode::aruco_marker_id_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_rate_),
      std::bind(&R1ArucoSerialNode::timer_callback, this));

    last_reconnect_attempt_ = this->now();
  }

  void timer_callback()
  {
    if (!serial_->is_connected()) {
      auto now = this->now();
      if ((now - last_reconnect_attempt_).seconds() >= reconnect_interval_sec_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Serial disconnected. Attempting reconnect to '%s'... (interval: %.1fs)",
          port_name_.c_str(), reconnect_interval_sec_);
        last_reconnect_attempt_ = now;
        if (serial_->reconnect()) {
          RCLCPP_INFO(this->get_logger(), "Reconnected to '%s' successfully.", port_name_.c_str());
          if (last_marker_id_.has_value()) {
            RCLCPP_INFO(
              this->get_logger(), "Resending last marker_id: %d", last_marker_id_.value());
            send_marker_id(last_marker_id_.value());
          }
        }
      }
      return;
    }

    std::vector<uint8_t> rx_buff = serial_->read();
    for (int i = 0; i < (int)rx_buff.size(); i++) {
      if (recv_index_ >= (int)sizeof(recv_buff_) - 1) {
        RCLCPP_WARN(this->get_logger(), "recv_buff_ overflow, resetting");
        recv_index_ = 0;
      }
      recv_buff_[recv_index_++] = rx_buff[i];
      if (rx_buff[i] == '\0') {
        RCLCPP_INFO(this->get_logger(), "%s", recv_buff_);
        recv_index_ = 0;
      }
    }
  }

  void aruco_marker_id_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    last_marker_id_ = msg->data;
    if (!serial_->is_connected()) return;
    send_marker_id(msg->data);
  }

  void send_marker_id(int marker_id)
  {
    std::string send_str = std::to_string(marker_id);
    std::vector<uint8_t> tx_buff(send_str.begin(), send_str.end());
    serial_->write_buff(tx_buff);
  }

  std::string port_name_;
  double timer_rate_ = 10.0;
  double reconnect_interval_sec_ = 1.0;
  rclcpp::Time last_reconnect_attempt_;
  std::optional<int> last_marker_id_;
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
