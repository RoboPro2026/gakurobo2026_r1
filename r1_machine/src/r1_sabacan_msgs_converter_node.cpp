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

#include "r1_msgs/msg/mecanum.hpp"
#include "r1_msgs/msg/motor.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_ref.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

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

    mecanum_wheel_speeds_ref_subscriber_ = this->create_subscription<r1_msgs::msg::Mecanum>(
      "/mecanum_wheel_speeds_ref", 10,
      std::bind(&MyNode::mecanum_wheel_speeds_ref_callback, this, std::placeholders::_1));

    mecanum_wheel_speeds_feedback_publisher_ =
      this->create_publisher<r1_msgs::msg::Mecanum>("/mecanum_wheel_speeds_feedback", 10);

    sabacan_robomas_status_subscription_ =
      this->create_subscription<sabacan_msgs::msg::SabacanRobomasStatus>(
        "/sabacan_robomas_status1", 10,
        std::bind(&MyNode::sabacan_robomas_status_callback, this, std::placeholders::_1));

    fl_motor_publisher_ = this->create_publisher<r1_msgs::msg::Motor>("/fl_motor", 10);
    fr_motor_publisher_ = this->create_publisher<r1_msgs::msg::Motor>("/fr_motor", 10);
    rl_motor_publisher_ = this->create_publisher<r1_msgs::msg::Motor>("/rl_motor", 10);
    rr_motor_publisher_ = this->create_publisher<r1_msgs::msg::Motor>("/rr_motor", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));

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

  void sabacan_robomas_status_callback(
    const sabacan_msgs::msg::SabacanRobomasStatus::ConstSharedPtr msg)
  {
    auto msg_status = r1_msgs::msg::Motor();
    msg_status.motor_type = msg->motor_type;
    msg_status.control_type = msg->control_type;
    msg_status.motor_state = msg->motor_state;
    msg_status.torque = msg->torque;
    msg_status.speed = msg->speed;
    msg_status.pos = msg->pos;
    msg_status.abs_pos = msg->abs_pos;
    msg_status.abs_speed = msg->abs_speed;
    msg_status.abs_turn_cnt = msg->abs_turn_cnt;
    msg_status.vesc_voltage = msg->vesc_voltage;
    msg_status.vesc_current = msg->vesc_current;
    msg_status.vesc_erpm = msg->vesc_erpm;
    // メカナムの運動学計算用にエンコーダの値を代入
    mecanum_wheel_speeds_feedback_[msg->motor_number] = msg->speed;

    if (msg->motor_number == FL) {
      fl_motor_publisher_->publish(msg_status);
    } else if (msg->motor_number == FR) {
      fr_motor_publisher_->publish(msg_status);
    } else if (msg->motor_number == RL) {
      rl_motor_publisher_->publish(msg_status);
    } else if (msg->motor_number == RR) {
      rr_motor_publisher_->publish(msg_status);
    }
  }

  void mecanum_wheel_speeds_ref_callback(const r1_msgs::msg::Mecanum::ConstSharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(), "wheel_speeds_ref: FL=%f, FR=%f, RL=%f, RR=%f", msg->fl_wheel_speed,
      msg->fr_wheel_speed, msg->rl_wheel_speed, msg->rr_wheel_speed);
    for (int i = 0; i < 4; i++) {
      auto msg_ref = sabacan_msgs::msg::SabacanRobomasRef();
      msg_ref.motor_number = mecanum_motor_number_[i];
      if (mecanum_motor_number_[i] == FL) {
        msg_ref.ref = msg->fl_wheel_speed;
      } else if (mecanum_motor_number_[i] == FR) {
        msg_ref.ref = msg->fr_wheel_speed;
      } else if (mecanum_motor_number_[i] == RL) {
        msg_ref.ref = msg->rl_wheel_speed;
      } else if (mecanum_motor_number_[i] == RR) {
        msg_ref.ref = msg->rr_wheel_speed;
      }
      sabacan_robomas_ref_publisher_[mecanum_board_id_[i]]->publish(msg_ref);
    }
  }

  void timer_callback()
  {
    // メカナムのフィードバック値を計算してパブリッシュ
    auto msg_feedback = r1_msgs::msg::Mecanum();
    msg_feedback.fl_wheel_speed = mecanum_wheel_speeds_feedback_[FL];
    msg_feedback.fr_wheel_speed = mecanum_wheel_speeds_feedback_[FR];
    msg_feedback.rl_wheel_speed = mecanum_wheel_speeds_feedback_[RL];
    msg_feedback.rr_wheel_speed = mecanum_wheel_speeds_feedback_[RR];
    mecanum_wheel_speeds_feedback_publisher_->publish(msg_feedback);
  }

  // sabacan_robomas_refの0~9までのpublisherを宣言
  std::vector<rclcpp::Publisher<sabacan_msgs::msg::SabacanRobomasRef>::SharedPtr>
    sabacan_robomas_ref_publisher_;
  rclcpp::Subscription<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_ref_subscriber_;
  rclcpp::Publisher<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_feedback_publisher_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr
    sabacan_robomas_status_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  // モータのパラメータ名
  int mecanum_board_id_[4];
  int mecanum_motor_number_[4];
  std::vector<double> mecanum_wheel_speeds_feedback_ = std::vector<double>(4);
  static constexpr int FL = 0;
  static constexpr int FR = 1;
  static constexpr int RL = 2;
  static constexpr int RR = 3;
  // デバッグ用
  rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr fl_motor_publisher_;
  rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr fr_motor_publisher_;
  rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr rl_motor_publisher_;
  rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr rr_motor_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
