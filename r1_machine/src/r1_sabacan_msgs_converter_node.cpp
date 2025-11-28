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
#include <vector>

#include "r1_msgs/msg/angle_motion.hpp"
#include "r1_msgs/msg/gpio_input.hpp"
#include "r1_msgs/msg/linear_motion.hpp"
#include "r1_msgs/msg/mecanum.hpp"
#include "r1_msgs/msg/motor.hpp"
#include "r1_msgs/msg/motor_ref.hpp"
#include "r1_msgs/msg/odometry_encoder.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_status.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "sabacan_single_control_msgs/msg/sabacan_robomas_single_ref.hpp"
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

    linear_motion_status_publisher_[X] =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/linear_motion_x_status", 10);
    linear_motion_status_publisher_[Y] =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/linear_motion_y_status", 10);
    angle_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::AngleMotion>("/angle_motion_status", 10);

    linear_motion_motor_ref_subscription_[X] = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/linear_motion_x_motor_ref", 10, [this](const r1_msgs::msg::MotorRef::SharedPtr msg) {
        this->linear_motion_motor_ref_callback(msg, X);
      });

    linear_motion_motor_ref_subscription_[Y] = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/linear_motion_y_motor_ref", 10, [this](const r1_msgs::msg::MotorRef::SharedPtr msg) {
        this->linear_motion_motor_ref_callback(msg, Y);
      });

    angle_motion_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/angle_motion_motor_ref", 10,
      std::bind(&MyNode::angle_motion_motor_ref_callback, this, std::placeholders::_1));

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

    auto gpio_number_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    gpio_number_descriptor.description =
      "The GPIO number on the CAN board. This parameter is mandatory.";
    gpio_number_descriptor.integer_range.resize(1);
    gpio_number_descriptor.integer_range[0].from_value = 0;
    gpio_number_descriptor.integer_range[0].to_value = 8;
    gpio_number_descriptor.integer_range[0].step = 1;

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
    this->declare_parameter<int>("linear_motion_x_board_id", board_id_descriptor);
    this->declare_parameter<int>("linear_motion_x_motor_number", number_descriptor);
    this->declare_parameter<int>("linear_motion_y_board_id", board_id_descriptor);
    this->declare_parameter<int>("linear_motion_y_motor_number", number_descriptor);
    this->declare_parameter<int>("linear_motion_x_low_switch_board_id", board_id_descriptor);
    this->declare_parameter<int>("linear_motion_x_high_switch_board_id", board_id_descriptor);
    this->declare_parameter<int>("linear_motion_y_low_switch_board_id", board_id_descriptor);
    this->declare_parameter<int>("linear_motion_y_high_switch_board_id", board_id_descriptor);
    this->declare_parameter<int>("linear_motion_x_low_switch_gpio_number", gpio_number_descriptor);
    this->declare_parameter<int>("linear_motion_x_high_switch_gpio_number", gpio_number_descriptor);
    this->declare_parameter<int>("linear_motion_y_low_switch_gpio_number", gpio_number_descriptor);
    this->declare_parameter<int>("linear_motion_y_high_switch_gpio_number", gpio_number_descriptor);
    this->declare_parameter<int>("angle_motion_board_id", board_id_descriptor);
    this->declare_parameter<int>("angle_motion_motor_number", number_descriptor);
    this->declare_parameter<int>("angle_motion_low_switch_board_id", board_id_descriptor);
    this->declare_parameter<int>("angle_motion_high_switch_board_id", board_id_descriptor);
    this->declare_parameter<int>("angle_motion_low_switch_gpio_number", gpio_number_descriptor);
    this->declare_parameter<int>("angle_motion_high_switch_gpio_number", gpio_number_descriptor);

    this->get_parameter("mecanum_fl_board_id", mecanum_[FL].board_id);
    this->get_parameter("mecanum_fr_board_id", mecanum_[FR].board_id);
    this->get_parameter("mecanum_rl_board_id", mecanum_[RL].board_id);
    this->get_parameter("mecanum_rr_board_id", mecanum_[RR].board_id);
    this->get_parameter("mecanum_fl_motor_number", mecanum_[FL].number);
    this->get_parameter("mecanum_fr_motor_number", mecanum_[FR].number);
    this->get_parameter("mecanum_rl_motor_number", mecanum_[RL].number);
    this->get_parameter("mecanum_rr_motor_number", mecanum_[RR].number);
    this->get_parameter("odometry_x_board_id", odometry_encoder_[X].board_id);
    this->get_parameter("odometry_y_board_id", odometry_encoder_[Y].board_id);
    this->get_parameter("odometry_x_motor_number", odometry_encoder_[X].number);
    this->get_parameter("odometry_y_motor_number", odometry_encoder_[Y].number);
    this->get_parameter("linear_motion_x_board_id", linear_motion_[X].board_id);
    this->get_parameter("linear_motion_y_board_id", linear_motion_[Y].board_id);
    this->get_parameter("linear_motion_x_motor_number", linear_motion_[X].number);
    this->get_parameter("linear_motion_y_motor_number", linear_motion_[Y].number);
    this->get_parameter(
      "linear_motion_x_low_switch_board_id", linear_motion_switch_[X][LOW].board_id);
    this->get_parameter(
      "linear_motion_x_high_switch_board_id", linear_motion_switch_[X][HIGH].board_id);
    this->get_parameter(
      "linear_motion_y_low_switch_board_id", linear_motion_switch_[Y][LOW].board_id);
    this->get_parameter(
      "linear_motion_y_high_switch_board_id", linear_motion_switch_[Y][HIGH].board_id);
    this->get_parameter(
      "linear_motion_x_low_switch_gpio_number", linear_motion_switch_[X][LOW].number);
    this->get_parameter(
      "linear_motion_x_high_switch_gpio_number", linear_motion_switch_[X][HIGH].number);
    this->get_parameter(
      "linear_motion_y_low_switch_gpio_number", linear_motion_switch_[Y][LOW].number);
    this->get_parameter(
      "linear_motion_y_high_switch_gpio_number", linear_motion_switch_[Y][HIGH].number);
    this->get_parameter("angle_motion_board_id", angle_motion_.board_id);
    this->get_parameter("angle_motion_motor_number", angle_motion_.number);
    this->get_parameter("angle_motion_low_switch_board_id", angle_motion_switch_[LOW].board_id);
    this->get_parameter("angle_motion_high_switch_board_id", angle_motion_switch_[HIGH].board_id);
    this->get_parameter("angle_motion_low_switch_gpio_number", angle_motion_switch_[LOW].number);
    this->get_parameter("angle_motion_high_switch_gpio_number", angle_motion_switch_[HIGH].number);

    // sabacan_single_control_msgsのpublisherを作成
    for (int i = 0; i < 4; ++i) {
      const auto & info = mecanum_[i];
      const std::string topic = "/sabacan_robomas_ref" + std::to_string(info.board_id) + "/motor" +
                                std::to_string(info.number);
      mecanum_single_ref_publisher_[i] =
        this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
          topic, 10);
    }

    for (size_t i = 0; i < linear_motion_.size(); ++i) {
      const auto & info = linear_motion_[i];
      const std::string topic = "/sabacan_robomas_ref" + std::to_string(info.board_id) + "/motor" +
                                std::to_string(info.number);
      linear_motion_single_ref_publisher_[i] =
        this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
          topic, 10);
    }

    {
      const auto & info = angle_motion_;
      const std::string topic = "/sabacan_robomas_ref" + std::to_string(info.board_id) + "/motor" +
                                std::to_string(info.number);
      angle_motion_single_ref_publisher_ =
        this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
          topic, 10);
    }

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
        odometry_encoder_[X], "odometry_x_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_odometry_x_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        odometry_encoder_[Y], "odometry_y_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_odometry_y_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        linear_motion_[X], "linear_motion_x_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_linear_motion_x_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        linear_motion_[Y], "linear_motion_y_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_linear_motion_y_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        angle_motion_, "angle_motion_motor_status",
        this->create_publisher<r1_msgs::msg::Motor>("/debug_angle_motion_motor_status", 10)});

    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        linear_motion_switch_[X][LOW], "linear_motion_x_low_switch_status",
        this->create_publisher<r1_msgs::msg::GPIOInput>(
          "/debug_linear_motion_x_low_switch_status", 10)});
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        linear_motion_switch_[X][HIGH], "linear_motion_x_high_switch_status",
        this->create_publisher<r1_msgs::msg::GPIOInput>(
          "/debug_linear_motion_x_high_switch_status", 10)});
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        linear_motion_switch_[Y][LOW], "linear_motion_y_low_switch_status",
        this->create_publisher<r1_msgs::msg::GPIOInput>(
          "/debug_linear_motion_y_low_switch_status", 10)});
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        linear_motion_switch_[Y][HIGH], "linear_motion_y_high_switch_status",
        this->create_publisher<r1_msgs::msg::GPIOInput>(
          "/debug_linear_motion_y_high_switch_status", 10)});
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        angle_motion_switch_[LOW], "angle_motion_low_switch_status",
        this->create_publisher<r1_msgs::msg::GPIOInput>(
          "/debug_angle_motion_low_switch_status", 10)});
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        angle_motion_switch_[HIGH], "angle_motion_high_switch_status",
        this->create_publisher<r1_msgs::msg::GPIOInput>(
          "/debug_angle_motion_high_switch_status", 10)});
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
    if (receive == odometry_encoder_[X]) {
      odometry_encoder_update_[X] = true;
      odometry_encoder_pos_values_[X] = msg->abs_pos;
      odometry_encoder_speed_values_[X] = msg->abs_speed;
    }
    if (receive == odometry_encoder_[Y]) {
      odometry_encoder_update_[Y] = true;
      odometry_encoder_pos_values_[Y] = msg->abs_pos;
      odometry_encoder_speed_values_[Y] = msg->abs_speed;
    }
    for (size_t i = 0; i < linear_motion_.size(); ++i) {
      if (receive == linear_motion_[i]) {
        auto linear_msg = r1_msgs::msg::LinearMotion();
        linear_msg.torque = msg->torque;
        linear_msg.speed = msg->speed;
        linear_msg.pos = msg->pos;
        linear_msg.low_switch = linear_motion_switch_value_[i][LOW];
        linear_msg.high_switch = linear_motion_switch_value_[i][HIGH];
        linear_motion_status_publisher_[i]->publish(linear_msg);
        linear_motion_value_[i] = msg->pos;
      }
    }

    if (receive == angle_motion_) {
      auto angle_msg = r1_msgs::msg::AngleMotion();
      angle_msg.torque = msg->torque;
      angle_msg.speed = msg->speed;
      angle_msg.pos = msg->pos;
      angle_msg.low_switch = angle_motion_switch_value_[LOW];
      angle_msg.high_switch = angle_motion_switch_value_[HIGH];
      angle_motion_status_publisher_->publish(angle_msg);
    }

    if (odometry_encoder_update_[X] && odometry_encoder_update_[Y]) {
      auto odom_msg = r1_msgs::msg::OdometryEncoder();
      odom_msg.encoder_pos_x = odometry_encoder_pos_values_[X];
      odom_msg.encoder_pos_y = odometry_encoder_pos_values_[Y];
      odom_msg.encoder_speed_x = odometry_encoder_speed_values_[X];
      odom_msg.encoder_speed_y = odometry_encoder_speed_values_[Y];
      odometry_encoder_publisher_->publish(odom_msg);
      odometry_encoder_update_[X] = false;
      odometry_encoder_update_[Y] = false;
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
    BoardInfo receive{board_id, msg->pin_number};

    // リニアモーション(X/Y)と角度モーションのスイッチ状態を更新
    for (size_t axis = 0; axis < linear_motion_switch_.size(); ++axis) {
      for (size_t sw = 0; sw < linear_motion_switch_[axis].size(); ++sw) {
        if (receive == linear_motion_switch_[axis][sw]) {
          linear_motion_switch_value_[axis][sw] = msg->input;
        }
      }
    }
    for (int sw = 0; sw < 2; ++sw) {
      if (receive == angle_motion_switch_[sw]) {
        angle_motion_switch_value_[sw] = msg->input;
      }
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
    for (int i = 0; i < MECANUM_NUM; i++) {
      auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
      msg_ref.control_type = "VELOCITY";

      if (mecanum_[i] == mecanum_[FL]) {
        msg_ref.ref = msg->fl_wheel_speed;
      } else if (mecanum_[i] == mecanum_[FR]) {
        msg_ref.ref = msg->fr_wheel_speed;
      } else if (mecanum_[i] == mecanum_[RL]) {
        msg_ref.ref = msg->rl_wheel_speed;
      } else if (mecanum_[i] == mecanum_[RR]) {
        msg_ref.ref = msg->rr_wheel_speed;
      }

      if (mecanum_single_ref_publisher_[i]) {
        mecanum_single_ref_publisher_[i]->publish(msg_ref);
      }
    }
  }

  void linear_motion_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg, int axis)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;

    if (linear_motion_single_ref_publisher_[axis]) {
      linear_motion_single_ref_publisher_[axis]->publish(msg_ref);
    }
  }

  void angle_motion_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;

    if (angle_motion_single_ref_publisher_) {
      angle_motion_single_ref_publisher_->publish(msg_ref);
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

  std::vector<
    rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr>
    mecanum_single_ref_publisher_{MECANUM_NUM};
  std::vector<
    rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr>
    linear_motion_single_ref_publisher_{LINEAR_MOTION_NUM};
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    angle_motion_single_ref_publisher_;
  std::vector<rclcpp::Publisher<sabacan_msgs::msg::SabacanGPIOStatus>::SharedPtr>
    sabacan_gpio_ref_publisher_;

  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr>
    sabacan_robomas_status_subscription_;
  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIOStatus>::SharedPtr>
    sabacan_gpio_status_subscription_;

  rclcpp::Subscription<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_ref_subscriber_;
  rclcpp::Publisher<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_feedback_publisher_;
  rclcpp::Publisher<r1_msgs::msg::OdometryEncoder>::SharedPtr odometry_encoder_publisher_;
  std::vector<rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr>
    linear_motion_status_publisher_{LINEAR_MOTION_NUM};
  rclcpp::Publisher<r1_msgs::msg::AngleMotion>::SharedPtr angle_motion_status_publisher_;
  std::vector<rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr>
    linear_motion_motor_ref_subscription_{LINEAR_MOTION_NUM};
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr angle_motion_motor_ref_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  // モータのパラメータ名
  std::vector<BoardInfo> mecanum_{MECANUM_NUM};
  std::vector<BoardInfo> odometry_encoder_{ODOMETRY_NUM};
  std::vector<BoardInfo> linear_motion_{LINEAR_MOTION_NUM};
  std::vector<std::vector<BoardInfo>> linear_motion_switch_ =
    std::vector<std::vector<BoardInfo>>(2, std::vector<BoardInfo>(2));  // [axis][switch]
  BoardInfo angle_motion_{};
  std::vector<BoardInfo> angle_motion_switch_ = std::vector<BoardInfo>(2);
  std::vector<double> odometry_encoder_pos_values_ = std::vector<double>(ODOMETRY_NUM);
  std::vector<double> odometry_encoder_speed_values_ = std::vector<double>(ODOMETRY_NUM);
  std::vector<bool> odometry_encoder_update_ = std::vector<bool>(ODOMETRY_NUM, true);
  std::vector<double> linear_motion_value_ = std::vector<double>(LINEAR_MOTION_NUM);
  std::vector<std::vector<bool>> linear_motion_switch_value_ =
    std::vector<std::vector<bool>>(2, std::vector<bool>(2, false));
  std::vector<bool> angle_motion_switch_value_ = std::vector<bool>(2, false);
  std::string linear_motion_control_type_;
  std::vector<double> mecanum_wheel_speeds_feedback_ = std::vector<double>(MECANUM_NUM);
  static constexpr int LOW = 0;
  static constexpr int HIGH = 1;
  static constexpr int FL = 0;
  static constexpr int FR = 1;
  static constexpr int RL = 2;
  static constexpr int RR = 3;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int MECANUM_NUM = 4;
  static constexpr int LINEAR_MOTION_NUM = 2;
  static constexpr int ODOMETRY_NUM = 2;
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
