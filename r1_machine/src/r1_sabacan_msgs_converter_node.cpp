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

#include "r1_msgs/msg/gpio_input.hpp"
#include "r1_msgs/msg/linear_motion.hpp"
#include "r1_msgs/msg/mecanum.hpp"
#include "r1_msgs/msg/motor.hpp"
#include "r1_msgs/msg/odometry_encoder.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_status.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_ref.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

struct BoardInfo
{
  int board_id;
  int number;

  bool operator==(const BoardInfo & other) const
  {
    return (board_id == other.board_id) && (number == other.number);
  }

  bool operator!=(const BoardInfo & other) const { return !this->operator==(other); }
};

struct DebugMotorInfo
{
  BoardInfo board_info;
  std::string name;
  rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr motor_status_publisher;

  bool operator==(const DebugMotorInfo & other) const
  {
    return (board_info == other.board_info) && (name == other.name);
  }

  bool operator!=(const DebugMotorInfo & other) const { return !this->operator==(other); }
};

struct DebugGPIOInputInfo
{
  BoardInfo board_info;
  std::string name;
  rclcpp::Publisher<r1_msgs::msg::GPIOInput>::SharedPtr gpio_input_status_publisher;

  bool operator==(const DebugGPIOInputInfo & other) const
  {
    return (board_info == other.board_info) && (name == other.name);
  }

  bool operator!=(const DebugGPIOInputInfo & other) const { return !this->operator==(other); }
};

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_sabacan_msgs_converter_node")
  {
    // sabacan_robomas_refの0~9までのpublisherを作成
    sabacan_robomas_ref_publisher_.resize(10);
    for (size_t i = 0; i < sabacan_robomas_ref_publisher_.size(); i++) {
      sabacan_robomas_ref_publisher_[i] =
        this->create_publisher<sabacan_msgs::msg::SabacanRobomasRef>(
          "/sabacan_robomas_ref" + std::to_string(i), 10);
    }

    sabacan_gpio_ref_publisher_.resize(10);
    for (size_t i = 0; i < sabacan_gpio_ref_publisher_.size(); i++) {
      sabacan_gpio_ref_publisher_[i] = this->create_publisher<sabacan_msgs::msg::SabacanGPIOStatus>(
        "/sabacan_gpio_ref" + std::to_string(i), 10);
    }

    sabacan_robomas_status_subscription_.resize(10);
    for (size_t i = 0; i < sabacan_robomas_status_subscription_.size(); i++) {
      sabacan_robomas_status_subscription_[i] =
        this->create_subscription<sabacan_msgs::msg::SabacanRobomasStatus>(
          "/sabacan_robomas_status" + std::to_string(i), 10,
          [this, i](sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg) {
            this->sabacan_robomas_status_callback(i, msg);
          });
    }

    sabacan_gpio_status_subscription_.resize(10);
    for (size_t i = 0; i < sabacan_gpio_status_subscription_.size(); i++) {
      sabacan_gpio_status_subscription_[i] =
        this->create_subscription<sabacan_msgs::msg::SabacanGPIOStatus>(
          "/sabacan_gpio_status" + std::to_string(i), 10,
          [this, i](sabacan_msgs::msg::SabacanGPIOStatus::SharedPtr msg) {
            this->sabacan_gpio_status_callback(i, msg);
          });
    }

    mecanum_wheel_speeds_ref_subscriber_ = this->create_subscription<r1_msgs::msg::Mecanum>(
      "/mecanum_wheel_speeds_ref", 10,
      std::bind(&MyNode::mecanum_wheel_speeds_ref_callback, this, std::placeholders::_1));

    odometry_encoder_publisher_ =
      this->create_publisher<r1_msgs::msg::OdometryEncoder>("/odometry_encoder", 10);

    mecanum_wheel_speeds_feedback_publisher_ =
      this->create_publisher<r1_msgs::msg::Mecanum>("/mecanum_wheel_speeds_feedback", 10);

    linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/linear_motion_status", 10);

    linear_motion_motor_ref_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/linear_motion_motor_position_ref", 10,
      std::bind(&MyNode::linear_motion_motor_ref_callback, this, std::placeholders::_1));

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

    auto number_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    number_descriptor.description =
      "The motor number on the CAN board (0-3). This parameter is mandatory.";
    number_descriptor.integer_range.resize(1);
    number_descriptor.integer_range[0].from_value = 0;
    number_descriptor.integer_range[0].to_value = 3;
    number_descriptor.integer_range[0].step = 1;

    // ロボットの動作中にここのパラメータは変えない前提
    this->declare_parameter<int>("mecanum_fl_board_id", board_id_descriptor);
    this->declare_parameter<int>("mecanum_fr_board_id", board_id_descriptor);
    this->declare_parameter<int>("mecanum_rl_board_id", board_id_descriptor);
    this->declare_parameter<int>("mecanum_rr_board_id", board_id_descriptor);
    this->declare_parameter<int>("mecanum_fl_motor_number", number_descriptor);
    this->declare_parameter<int>("mecanum_fr_motor_number", number_descriptor);
    this->declare_parameter<int>("mecanum_rl_motor_number", number_descriptor);
    this->declare_parameter<int>("mecanum_rr_motor_number", number_descriptor);
    this->declare_parameter<int>("odometry_x_board_id", board_id_descriptor);
    this->declare_parameter<int>("odometry_y_board_id", board_id_descriptor);
    this->declare_parameter<int>("odometry_x_motor_number", number_descriptor);
    this->declare_parameter<int>("odometry_y_motor_number", number_descriptor);
    this->declare_parameter<int>("linear_motion_board_id", board_id_descriptor);
    this->declare_parameter<int>("linear_motion_motor_number", number_descriptor);
    this->declare_parameter<int>("linear_motion_low_switch_board_id", board_id_descriptor);
    this->declare_parameter<int>("linear_motion_high_switch_board_id", board_id_descriptor);
    this->declare_parameter<int>("linear_motion_low_switch_gpio_number", number_descriptor);
    this->declare_parameter<int>("linear_motion_high_switch_gpio_number", number_descriptor);

    this->get_parameter("mecanum_fl_board_id", mecanum_[0].board_id);
    this->get_parameter("mecanum_fr_board_id", mecanum_[1].board_id);
    this->get_parameter("mecanum_rl_board_id", mecanum_[2].board_id);
    this->get_parameter("mecanum_rr_board_id", mecanum_[3].board_id);
    this->get_parameter("mecanum_fl_motor_number", mecanum_[0].number);
    this->get_parameter("mecanum_fr_motor_number", mecanum_[1].number);
    this->get_parameter("mecanum_rl_motor_number", mecanum_[2].number);
    this->get_parameter("mecanum_rr_motor_number", mecanum_[3].number);
    this->get_parameter("odometry_x_board_id", odometry_encoder_[0].board_id);
    this->get_parameter("odometry_y_board_id", odometry_encoder_[1].board_id);
    this->get_parameter("odometry_x_motor_number", odometry_encoder_[0].number);
    this->get_parameter("odometry_y_motor_number", odometry_encoder_[1].number);
    this->get_parameter("linear_motion_board_id", linear_motion_.board_id);
    this->get_parameter("linear_motion_motor_number", linear_motion_.number);
    this->get_parameter("linear_motion_low_switch_board_id", linear_motion_switch_[0].board_id);
    this->get_parameter("linear_motion_high_switch_board_id", linear_motion_switch_[1].board_id);
    this->get_parameter("linear_motion_low_switch_gpio_number", linear_motion_switch_[0].number);
    this->get_parameter("linear_motion_high_switch_gpio_number", linear_motion_switch_[1].number);

    // デバッグ用のpublisherを追加
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[FL], "mecanum_fl_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_fl_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[FR], "mecanum_fr_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_fr_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[RL], "mecanum_rl_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_rl_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[RR], "mecanum_rr_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_rr_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        odometry_encoder_[0], "odometry_x_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_odometry_x_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        odometry_encoder_[1], "odometry_y_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_odometry_y_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        linear_motion_, "linear_motion_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_linear_motion_motor_status", 10)});

    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        linear_motion_switch_[0], "linear_motion_low_switch_status",
        this->create_publisher<r1_msgs::msg::GPIOInput>(
          "/debug_linear_motion_low_switch_status", 10)});
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        linear_motion_switch_[1], "linear_motion_high_switch_status",
        this->create_publisher<r1_msgs::msg::GPIOInput>(
          "/debug_linear_motion_high_switch_status", 10)});
  }

  bool board_info_match(BoardInfo a, BoardInfo b)
  {
    return a.board_id == b.board_id && a.number == b.number;
  }

  void sabacan_robomas_status_callback(
    int baord_id, sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg)
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

    BoardInfo receive{baord_id, msg->motor_number};

    if (receive == mecanum_[FL]) {
      mecanum_wheel_speeds_feedback_[msg->motor_number] = msg->speed;
    }
    if (receive == mecanum_[FR]) {
      mecanum_wheel_speeds_feedback_[msg->motor_number] = msg->speed;
    }
    if (receive == mecanum_[RL]) {
      mecanum_wheel_speeds_feedback_[msg->motor_number] = msg->speed;
    }
    if (receive == mecanum_[RR]) {
      mecanum_wheel_speeds_feedback_[msg->motor_number] = msg->speed;
    }
    if (receive == odometry_encoder_[0]) {
      odometry_encoder_update_[0] = true;
      odometry_encoder_pos_values_[0] = msg->abs_pos;
      odometry_encoder_speed_values_[0] = msg->abs_speed;
    }
    if (receive == odometry_encoder_[1]) {
      odometry_encoder_update_[1] = true;
      odometry_encoder_pos_values_[1] = msg->abs_pos;
      odometry_encoder_speed_values_[1] = msg->abs_speed;
    }
    if (receive == linear_motion_) {
      auto linear_msg = r1_msgs::msg::LinearMotion();
      linear_msg.pos = msg->pos;
      linear_msg.low_switch = linear_motion_switch_value_[0];
      linear_msg.high_switch = linear_motion_switch_value_[1];
      linear_motion_status_publisher_->publish(linear_msg);
      linear_motion_value_ = msg->pos;
    }

    if (odometry_encoder_update_[0] && odometry_encoder_update_[1]) {
      auto odom_msg = r1_msgs::msg::OdometryEncoder();
      odom_msg.encoder_pos_x = odometry_encoder_pos_values_[0];
      odom_msg.encoder_pos_y = odometry_encoder_pos_values_[1];
      odom_msg.encoder_speed_x = odometry_encoder_speed_values_[0];
      odom_msg.encoder_speed_y = odometry_encoder_speed_values_[1];
      odometry_encoder_publisher_->publish(odom_msg);
      odometry_encoder_update_[0] = false;
      odometry_encoder_update_[1] = false;
    }

    // デバッグ用パブリッシュ（マッチするものに配信）
    for (size_t i = 0; i < debug_motor_.size(); ++i) {
      const auto & dbg = debug_motor_[i];
      if (receive == dbg.board_info && dbg.motor_status_publisher) {
        dbg.motor_status_publisher->publish(msg_status);
      }
    }
  }

  void sabacan_gpio_status_callback(
    int board_id, sabacan_msgs::msg::SabacanGPIOStatus::SharedPtr msg)
  {
    auto msg_status = sabacan_msgs::msg::SabacanGPIOStatus();
    msg_status.pin_number = msg->pin_number;
    msg_status.input = msg->input;

    BoardInfo receive{board_id, msg->pin_number};

    // リニアモーションのスイッチ状態を更新
    if (receive == linear_motion_switch_[0]) {
      linear_motion_switch_value_[0] = msg->input;
    }
    if (receive == linear_motion_switch_[1]) {
      linear_motion_switch_value_[1] = msg->input;
    }

    // デバッグ用パブリッシュ（GPIO入力）
    r1_msgs::msg::GPIOInput gpio_msg;
    gpio_msg.status = msg->input;
    for (size_t i = 0; i < debug_gpio_input_.size(); ++i) {
      const auto & dbg = debug_gpio_input_[i];
      if (receive == dbg.board_info && dbg.gpio_input_status_publisher) {
        dbg.gpio_input_status_publisher->publish(gpio_msg);
      }
    }
  }

  void mecanum_wheel_speeds_ref_callback(const r1_msgs::msg::Mecanum::ConstSharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(), "wheel_speeds_ref: FL=%f, FR=%f, RL=%f, RR=%f", msg->fl_wheel_speed,
      msg->fr_wheel_speed, msg->rl_wheel_speed, msg->rr_wheel_speed);
    for (int i = 0; i < 4; i++) {
      auto msg_ref = sabacan_msgs::msg::SabacanRobomasRef();
      msg_ref.motor_number = mecanum_[i].number;

      // board_idとmotor_numberがマッチしたものを、指令値更新
      if (mecanum_[i] == mecanum_[FL]) {
        msg_ref.ref = msg->fl_wheel_speed;
      } else if (mecanum_[i] == mecanum_[FR]) {
        msg_ref.ref = msg->fr_wheel_speed;
      } else if (mecanum_[i] == mecanum_[RL]) {
        msg_ref.ref = msg->rl_wheel_speed;
      } else if (mecanum_[i] == mecanum_[RR]) {
        msg_ref.ref = msg->rr_wheel_speed;
      }

      sabacan_robomas_ref_publisher_[mecanum_[i].board_id]->publish(msg_ref);
    }
  }

  void linear_motion_motor_ref_callback(const std_msgs::msg::Float64::ConstSharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "linear_motion_motor_ref: %f", msg->data);
    auto msg_ref = sabacan_msgs::msg::SabacanRobomasRef();
    msg_ref.motor_number = linear_motion_.number;
    msg_ref.ref = msg->data;
    sabacan_robomas_ref_publisher_[linear_motion_.board_id]->publish(msg_ref);
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

  std::vector<rclcpp::Publisher<sabacan_msgs::msg::SabacanRobomasRef>::SharedPtr>
    sabacan_robomas_ref_publisher_;
  std::vector<rclcpp::Publisher<sabacan_msgs::msg::SabacanGPIOStatus>::SharedPtr>
    sabacan_gpio_ref_publisher_;

  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr>
    sabacan_robomas_status_subscription_;
  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIOStatus>::SharedPtr>
    sabacan_gpio_status_subscription_;

  rclcpp::Subscription<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_ref_subscriber_;
  rclcpp::Publisher<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_feedback_publisher_;
  rclcpp::Publisher<r1_msgs::msg::OdometryEncoder>::SharedPtr odometry_encoder_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr linear_motion_status_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr linear_motion_motor_ref_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  // モータのパラメータ名
  BoardInfo mecanum_[4];
  BoardInfo odometry_encoder_[2];
  BoardInfo linear_motion_;
  BoardInfo linear_motion_switch_[2];
  double odometry_encoder_pos_values_[2] = {0.0, 0.0};
  double odometry_encoder_speed_values_[2] = {0.0, 0.0};
  bool odometry_encoder_update_[2] = {true, true};
  double linear_motion_value_ = 0.0;
  bool linear_motion_switch_value_[2] = {false, false};
  std::vector<double> mecanum_wheel_speeds_feedback_ = std::vector<double>(4);
  static constexpr int FL = 0;
  static constexpr int FR = 1;
  static constexpr int RL = 2;
  static constexpr int RR = 3;
  // デバッグ用
  std::vector<DebugMotorInfo> debug_motor_;
  std::vector<DebugGPIOInputInfo> debug_gpio_input_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
