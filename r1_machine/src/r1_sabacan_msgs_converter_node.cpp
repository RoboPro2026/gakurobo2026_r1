/**
 * @file r1_sabacan_msgs_converter_node.cpp
 * @author Yamaguchi Yudai
 * @brief sabacan_msgsとr1_machineのノード間でのメッセージ変換を行うノード。いわゆる汚い処理を一箇所にまとめたもの。
 * @version 0.1
 * @date 2025-10-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <chrono>
#include <limits>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_ref.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_float64_to_sabacan_converter")
  {
    // sabacan_robomas_refの0~9までのpublisherを作成
    sabacan_robomas_ref_publisher_.resize(10);
    for (size_t i = 0; i < sabacan_robomas_ref_publisher_.size(); i++) {
      sabacan_robomas_ref_publisher_[i] =
        this->create_publisher<sabacan_msgs::msg::SabacanRobomasRef>(
          "/sabacan_robomas_ref" + std::to_string(i), 10);
    }

    mecanum_wheel_speeds_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mecanum_wheel_speeds", 10,
      std::bind(&MyNode::mecanum_wheel_speeds_callback, this, std::placeholders::_1));

    // パラメータの宣言
    // パラメータはすべて初期値を明確にしないと動かない。
    auto board_id_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    board_id_descriptor.description =
      "The unique ID of the CAN board (0-9). This parameter is mandatory.";
    board_id_descriptor.integer_range.resize(1);
    board_id_descriptor.integer_range[0].from_value = 0;
    board_id_descriptor.integer_range[0].to_value = 9;
    board_id_descriptor.integer_range[0].step = 1;

    auto motor_number_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    motor_number_descriptor.description =
      "The motor number on the CAN board (0-3). This parameter is mandatory.";
    motor_number_descriptor.integer_range.resize(1);
    motor_number_descriptor.integer_range[0].from_value = 0;
    motor_number_descriptor.integer_range[0].to_value = 3;
    motor_number_descriptor.integer_range[0].step = 1;

    this->declare_parameter<int>("mecanum_fl_board_id", board_id_descriptor);
    this->declare_parameter<int>("mecanum_fr_board_id", board_id_descriptor);
    this->declare_parameter<int>("mecanum_rl_board_id", board_id_descriptor);
    this->declare_parameter<int>("mecanum_rr_board_id", board_id_descriptor);
    this->declare_parameter<int>("mecanum_fl_motor_number", motor_number_descriptor);
    this->declare_parameter<int>("mecanum_fr_motor_number", motor_number_descriptor);
    this->declare_parameter<int>("mecanum_rl_motor_number", motor_number_descriptor);
    this->declare_parameter<int>("mecanum_rr_motor_number", motor_number_descriptor);

    this->get_parameter("mecanum_fl_board_id", mecanum_board_id_[0]);
    this->get_parameter("mecanum_fr_board_id", mecanum_board_id_[1]);
    this->get_parameter("mecanum_rl_board_id", mecanum_board_id_[2]);
    this->get_parameter("mecanum_rr_board_id", mecanum_board_id_[3]);
    this->get_parameter("mecanum_fl_motor_number", mecanum_motor_number_[0]);
    this->get_parameter("mecanum_fr_motor_number", mecanum_motor_number_[1]);
    this->get_parameter("mecanum_rl_motor_number", mecanum_motor_number_[2]);
    this->get_parameter("mecanum_rr_motor_number", mecanum_motor_number_[3]);
  }

  void mecanum_wheel_speeds_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_ERROR(this->get_logger(), "wheel_speeds size error: size=%ld", msg->data.size());
      return;
    }

    RCLCPP_DEBUG(
      this->get_logger(), "wheel_speeds: FL=%f, FR=%f, RL=%f, RR=%f", msg->data[0], msg->data[1],
      msg->data[2], msg->data[3]);
    std::vector<double> wheel_speeds = msg->data;
    for (int i = 0; i < 4; i++) {
      auto msg = sabacan_msgs::msg::SabacanRobomasRef();
      msg.motor_number = mecanum_motor_number_[i];
      msg.ref = wheel_speeds[i];
      sabacan_robomas_ref_publisher_[mecanum_board_id_[i]]->publish(msg);
    }
  }

  // sabacan_robomas_refの0~9までのpublisherを宣言
  std::vector<rclcpp::Publisher<sabacan_msgs::msg::SabacanRobomasRef>::SharedPtr>
    sabacan_robomas_ref_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    mecanum_wheel_speeds_subscriber_;
  // モータのパラメータ名
  int mecanum_board_id_[4];
  int mecanum_motor_number_[4];
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}