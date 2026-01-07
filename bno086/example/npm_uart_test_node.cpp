/**
 * @file npm_uart_test_node.cpp
 * @author Yamaguchi Yudai
 * @brief bno086のシーケンス番号を読んで、パケットロスがないことを確認するノード
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/*
実行方法
# /dev/ttyUSB0は必要に応じて名前を変えること
sudo chmod 666 /dev/ttyUSB0

# FT234だと、デフォルトだと16msの間バッファにデータを貯めるので、1msで送信するようにする

echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

ros2 run bno086 npm_uart_test_node --ros-args -p port:="/dev/ttyUSB0"

解説
PACKET_SIZE回分のシーケンス番号を保存し、最後のデータを受信したときにパケットロスをチェックする。
シーケンス番号が連続していなければパケットロスとカウントし、受信成功率を計算して表示する。
もし、2秒間データが受信できなければエラーメッセージを表示する。
*/

#include <chrono>

#include "bno086/bno086_driver.h"
#include "bno086/serial_driver.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("npm_uart_test_node")
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
    // 1kHz
    // 本来は100Hzあれば十分だが、パケットロス確認のために早くしている。（ポーリング）
    timer_publisher_ = this->create_wall_timer(1ms, std::bind(&MyNode::timer_callback, this));
  }

  void timer_callback(void)
  {
    // 新しいデータが来ていれば更新
    if (bno086_driver_->update()) {
      rx_data[index] = bno086_driver_->get_data().index;
      last_time_ = this->now();
      // 配列の最後のデータのときはパケットロスをチェックし、受信成功率を計算
      if (index == PACKET_SIZE - 1) {
        int loss_cnt = 0;
        for (int i = 1; i < PACKET_SIZE; i++) {
          // シーケンス番号が連続していなければ、パケットロスとカウントする。255の次は0なので、その場合も考慮している。
          if (rx_data[i] != (rx_data[i - 1] + 1) % 256) {
            loss_cnt++;
          }
        }
        // 受信成功率を表示
        double success_rate = (double)(PACKET_SIZE - loss_cnt) / PACKET_SIZE * 100.0;
        RCLCPP_INFO(
          this->get_logger(), "UART Received %d packets, Loss: %d packets, Success rate: %.2f%%",
          PACKET_SIZE, loss_cnt, success_rate);
      }
      // インデックスを更新
      index = (index + 1) % PACKET_SIZE;
      auto data = bno086_driver_->get_data();
      // RCLCPP_INFO(this->get_logger(), "Index: %d", data.index);
    }
    if ((this->now() - last_time_).seconds() > 2.0) {
      RCLCPP_ERROR(this->get_logger(), "No data received few second.");
    }
  }

  std::shared_ptr<SerialDriver> serial_;
  std::shared_ptr<BNO086Driver> bno086_driver_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;
  rclcpp::Time last_time_ = this->now();

  int PACKET_SIZE = 100;
  std::vector<int> rx_data = std::vector<int>(PACKET_SIZE, -1);
  int index = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}