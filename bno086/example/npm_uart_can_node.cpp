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

#include "bno086/serial_driver.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class UartCAN
{
private:
  std::shared_ptr<SerialDriver> serial_;

public:
  //バッファ
  std::vector<uint8_t> buff_;
  uint32_t seq_id_;
  static const int BUFF_SIZE = 256;

public:
  UartCAN(std::shared_ptr<SerialDriver> serial) : serial_(serial) { buff_.resize(BUFF_SIZE); }

  uint8_t calc_check_sum()
  {
    // データシートより
    // Checksum (Csum): The Index, yaw, pitch, roll, acceleration and reserved data bytes are added to produce the
    // checksum.
    int ret = 0;
    // TODO: 本当はi=2だけど、STM32間違えたので、i=1から計算
    for (int i = 1; i < 15; i++) ret += buff_[i];
    return ret & 0xff;
  }

  bool decode(std::vector<uint8_t> rx_buff)
  {
    static int i = 0;
    int j = 0;
    bool is_update = false;
    // 受信データを受信状況に応じて処理
    while (j < (int)rx_buff.size()) {
      switch (i) {
        case 0:
          buff_[i] = rx_buff[j];
          if ((buff_[0] = rx_buff[j]) == 0xAA) {
            i++;
          }
          break;
        case 1:
          buff_[i] = rx_buff[j];
          if ((buff_[1] = rx_buff[j]) == 0xAA) {
            i++;
          } else {
            i = 0;
          }
          break;
        case 18:
          // チェックサムを計算
          buff_[i] = rx_buff[j];
          if (buff_[15] == calc_check_sum()) {
            // 値を更新
            seq_id_ = (uint32_t)buff_[7] | ((uint32_t)buff_[8] << 8) | ((uint32_t)buff_[9] << 16) |
                      ((uint32_t)buff_[10] << 24);
            is_update = true;
          } else {
          }
          i = 0;
          break;
        default:
          buff_[i] = rx_buff[j];
          i++;
          break;
      }
      j++;
    }
    return is_update;
  }

  bool update()
  {
    std::vector<uint8_t> rx_buff = serial_->read();
    bool is_update = decode(rx_buff);
    return is_update;
  }
};

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("npm_uart_can_node")
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
    uart_can_ = std::make_shared<UartCAN>(serial_);
    // 1kHz
    // 本来は100Hzあれば十分だが、パケットロス確認のために早くしている。（ポーリング）
    timer_publisher_ = this->create_wall_timer(1ms, std::bind(&MyNode::timer_callback, this));
  }

  void timer_callback(void)
  {
    // 新しいデータが来ていれば更新

    if ((this->now() - start_time_).seconds() > 10.0) {
      int loss_cnt = 0;
      for (int i = 1; i < rx_data.size(); i++) {
        // シーケンス番号が連続していなければ、パケットロスとカウントする
        if (rx_data[i] != rx_data[i - 1] + 1) {
          loss_cnt++;
        }
      }
      // 受信成功率を表示
      if (rx_data.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "No data received");
        exit(0);
      }
      double success_rate = (double)(rx_data.size() - loss_cnt) / (double)rx_data.size() * 100.0;
      RCLCPP_INFO(
        this->get_logger(), "CAN Received %d packets, Loss: %d packets, Success rate: %.2f%%",
        rx_data.size(), loss_cnt, success_rate);
      exit(0);
    }

    if (uart_can_->update()) {
      rx_data.push_back(uart_can_->seq_id_);
      RCLCPP_INFO(this->get_logger(), "Received CAN seq_id: %u", uart_can_->seq_id_);
    }
  }

  std::shared_ptr<SerialDriver> serial_;
  std::shared_ptr<UartCAN> uart_can_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;
  rclcpp::Time start_time_ = this->now();

  std::vector<int> rx_data;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}