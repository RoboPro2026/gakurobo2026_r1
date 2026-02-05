/**
 * @file r1_sabacan_msgs_converter_node.cpp
 * @author Yamaguchi Yudai
 * @brief sabacan_msgsとr1_machineのノード間でのメッセージ変換を行うノード。
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
#include "r1_msgs/msg/gpio_pwm_ref.hpp"
#include "r1_msgs/msg/gpio_servo_ref.hpp"
#include "r1_msgs/msg/linear_motion.hpp"
#include "r1_msgs/msg/mecanum.hpp"
#include "r1_msgs/msg/motor.hpp"
#include "r1_msgs/msg/motor_ref.hpp"
#include "r1_msgs/msg/odometry_encoder.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_ref_float.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_ref_int.hpp"
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
  rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr motor_status_publisher;

  bool operator==(const DebugMotorInfo & other) const { return board_info == other.board_info; }

  bool operator!=(const DebugMotorInfo & other) const { return !this->operator==(other); }
};

struct DebugGPIOInputInfo
{
  BoardInfo board_info;
  rclcpp::Publisher<r1_msgs::msg::GpioInput>::SharedPtr gpio_input_status_publisher;

  bool operator==(const DebugGPIOInputInfo & other) const { return board_info == other.board_info; }

  bool operator!=(const DebugGPIOInputInfo & other) const { return !this->operator==(other); }
};

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_sabacan_msgs_converter_node")
  {
    // ========== Publisherとsubscriptionの作成 ==========
    // sabacan_gpio_ref_publisher_の作成
    sabacan_gpio_ref_int_publisher_.resize(10);
    sabacan_gpio_ref_float_publisher_.resize(10);
    for (size_t i = 0; i < 10; i++) {
      sabacan_gpio_ref_int_publisher_[i] =
        this->create_publisher<sabacan_msgs::msg::SabacanGPIORefInt>(
          "/sabacan_gpio_ref_int" + std::to_string(i), 10);
      sabacan_gpio_ref_float_publisher_[i] =
        this->create_publisher<sabacan_msgs::msg::SabacanGPIORefFloat>(
          "/sabacan_gpio_ref_float" + std::to_string(i), 10);
    }

    // sabacan_robomas_status_subscription_の作成
    sabacan_robomas_status_subscription_.resize(10);
    for (size_t i = 0; i < sabacan_robomas_status_subscription_.size(); i++) {
      sabacan_robomas_status_subscription_[i] =
        this->create_subscription<sabacan_msgs::msg::SabacanRobomasStatus>(
          "/sabacan_robomas_status" + std::to_string(i), 10,
          [this, i](sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg) {
            this->sabacan_robomas_status_callback(i, msg);
          });
    }

    // sabacan_gpio_status_subscription_の作成
    sabacan_gpio_status_subscription_.resize(10);
    for (size_t i = 0; i < sabacan_gpio_status_subscription_.size(); i++) {
      sabacan_gpio_status_subscription_[i] =
        this->create_subscription<sabacan_msgs::msg::SabacanGPIOStatus>(
          "/sabacan_gpio_status" + std::to_string(i), 10,
          [this, i](sabacan_msgs::msg::SabacanGPIOStatus::SharedPtr msg) {
            this->sabacan_gpio_status_callback(i, msg);
          });
    }
    // メカナム
    mecanum_wheel_speeds_ref_subscription_ = this->create_subscription<r1_msgs::msg::Mecanum>(
      "/mecanum_wheel_speeds_ref", 10,
      std::bind(&MyNode::mecanum_wheel_speeds_ref_callback, this, std::placeholders::_1));
    mecanum_wheel_speeds_feedback_publisher_ =
      this->create_publisher<r1_msgs::msg::Mecanum>("/mecanum_wheel_speeds_feedback", 10);

    // オドメトリ
    odometry_encoder_publisher_ =
      this->create_publisher<r1_msgs::msg::OdometryEncoder>("/odometry_encoder", 10);

    // KFS回収機構の指令値
    kfs_fx_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_fx_motor_ref", 10,
      std::bind(&MyNode::kfs_fx_motor_ref_callback, this, std::placeholders::_1));
    kfs_fz_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_fz_motor_ref", 10,
      std::bind(&MyNode::kfs_fz_motor_ref_callback, this, std::placeholders::_1));
    kfs_fyaw_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_fyaw_motor_ref", 10,
      std::bind(&MyNode::kfs_fyaw_motor_ref_callback, this, std::placeholders::_1));
    kfs_rx_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_rx_motor_ref", 10,
      std::bind(&MyNode::kfs_rx_motor_ref_callback, this, std::placeholders::_1));
    kfs_rz_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_rz_motor_ref", 10,
      std::bind(&MyNode::kfs_rz_motor_ref_callback, this, std::placeholders::_1));
    kfs_ryaw_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_ryaw_motor_ref", 10,
      std::bind(&MyNode::kfs_ryaw_motor_ref_callback, this, std::placeholders::_1));
    // KFS回収機構のフィードバック
    kfs_fx_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/kfs_fx_linear_motion_status", 10);
    kfs_fz_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/kfs_fz_linear_motion_status", 10);
    kfs_fyaw_angle_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::AngleMotion>("/kfs_fyaw_angle_motion_status", 10);
    kfs_rx_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/kfs_rx_linear_motion_status", 10);
    kfs_rz_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/kfs_rz_linear_motion_status", 10);
    kfs_ryaw_angle_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::AngleMotion>("/kfs_ryaw_angle_motion_status", 10);

    // 展開の指令値
    front_expand_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/front_expand_motor_ref", 10,
      std::bind(&MyNode::front_expand_motor_ref_callback, this, std::placeholders::_1));
    rear_expand_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/rear_expand_motor_ref", 10,
      std::bind(&MyNode::rear_expand_motor_ref_callback, this, std::placeholders::_1));
    // 展開のフィードバック
    front_expand_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/front_expand_linear_motion_status", 10);
    rear_expand_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/rear_expand_linear_motion_status", 10);

    // R2昇降の指令値
    r2_lift_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/r2_lift_motor_ref", 10,
      std::bind(&MyNode::r2_lift_motor_ref_callback, this, std::placeholders::_1));
    // R2昇降のフィードバック
    r2_lift_motor_status_publisher_ =
      this->create_publisher<r1_msgs::msg::Motor>("/r2_lift_motor_status", 10);

    //KFS真空ポンプ
    kfs_front_pump_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/kfs_front_pump_gpio_pwm_ref", 10,
      std::bind(&MyNode::kfs_front_pump_gpio_pwm_ref_callback, this, std::placeholders::_1));
    kfs_rear_pump_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/kfs_rear_pump_gpio_pwm_ref", 10,
      std::bind(&MyNode::kfs_rear_pump_gpio_pwm_ref_callback, this, std::placeholders::_1));

    // 真空電磁弁
    kfs_front_valve_gpio_pwm_ref_subscription_ =
      this->create_subscription<r1_msgs::msg::GpioPwmRef>(
        "/kfs_front_valve_gpio_pwm_ref", 10,
        std::bind(&MyNode::kfs_front_valve_gpio_pwm_ref_callback, this, std::placeholders::_1));
    kfs_rear_valve_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/kfs_rear_valve_gpio_pwm_ref", 10,
      std::bind(&MyNode::kfs_rear_valve_gpio_pwm_ref_callback, this, std::placeholders::_1));

    // KFSリミットスイッチ
    kfs_front_switch_status_publisher_ =
      this->create_publisher<r1_msgs::msg::GpioInput>("/kfs_front_switch_status", 10);
    kfs_rear_switch_status_publisher_ =
      this->create_publisher<r1_msgs::msg::GpioInput>("/kfs_rear_switch_status", 10);

    // ========== sabacan_single_control_msgsのpublisherを作成 =========
    // メカナム
    for (int i = 0; i < 4; ++i) {
      const auto & info = mecanum_[i];
      const std::string topic = "/sabacan_robomas_ref" + std::to_string(info.board_id) + "/motor" +
                                std::to_string(info.number);
      mecanum_single_ref_publisher_[i] =
        this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
          topic, 10);
    }
    // KFS回収機構
    kfs_fx_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "sabacan_robomas_ref" + std::to_string(kfs_fx_.board_id) + "/motor" +
          std::to_string(kfs_fx_.number),
        10);
    kfs_fz_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_fz_.board_id) + "/motor" +
          std::to_string(kfs_fz_.number),
        10);
    kfs_fyaw_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_fyaw_.board_id) + "/motor" +
          std::to_string(kfs_fyaw_.number),
        10);
    kfs_rx_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_rx_.board_id) + "/motor" +
          std::to_string(kfs_rx_.number),
        10);
    kfs_rz_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_rz_.board_id) + "/motor" +
          std::to_string(kfs_rz_.number),
        10);
    kfs_ryaw_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_ryaw_.board_id) + "/motor" +
          std::to_string(kfs_ryaw_.number),
        10);
    // 展開
    front_expand_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(front_expand_.board_id) + "/motor" +
          std::to_string(front_expand_.number),
        10);
    rear_expand_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(rear_expand_.board_id) + "/motor" +
          std::to_string(rear_expand_.number),
        10);
    // R2昇降
    r2_lift_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(r2_lift_.board_id) + "/motor" +
          std::to_string(r2_lift_.number),
        10);

    // デバッグ用のpublisherを追加
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[FL],
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_fl_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[FR],
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_fr_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[RL],
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_rl_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[RR],
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_rr_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_fx_, this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_fx_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_fz_, this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_fz_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_fyaw_,
        this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_fyaw_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_rx_, this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_rx_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_rz_, this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_rz_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_ryaw_,
        this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_ryaw_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        r2_lift_, this->create_publisher<r1_msgs::msg::Motor>("/debug_r2_lift_motor_status", 10)});

    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        kfs_front_switch_,
        this->create_publisher<r1_msgs::msg::GpioInput>("/debug_kfs_front_switch_status", 10)});
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        kfs_rear_switch_,
        this->create_publisher<r1_msgs::msg::GpioInput>("/debug_kfs_rear_switch_status", 10)});

    // 100Hzのタイマーを作成
    timer_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));
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
    msg_status.vesc_speed = msg->vesc_speed;

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

    if (receive == kfs_fx_) {
      auto linear_msg = r1_msgs::msg::LinearMotion();
      linear_msg.torque = msg->torque;
      linear_msg.speed = msg->speed;
      linear_msg.pos = msg->pos;
      kfs_fx_linear_motion_status_publisher_->publish(linear_msg);
    }

    if (receive == kfs_fz_) {
      auto linear_msg = r1_msgs::msg::LinearMotion();
      linear_msg.torque = msg->torque;
      linear_msg.speed = msg->speed;
      linear_msg.pos = msg->pos;
      kfs_fz_linear_motion_status_publisher_->publish(linear_msg);
    }

    if (receive == kfs_fyaw_) {
      auto angle_msg = r1_msgs::msg::AngleMotion();
      angle_msg.torque = msg->torque;
      angle_msg.speed = msg->speed;
      angle_msg.pos = msg->pos;
      kfs_fyaw_angle_motion_status_publisher_->publish(angle_msg);
    }

    if (receive == kfs_rx_) {
      auto linear_msg = r1_msgs::msg::LinearMotion();
      linear_msg.torque = msg->torque;
      linear_msg.speed = msg->speed;
      linear_msg.pos = msg->pos;
      kfs_rx_linear_motion_status_publisher_->publish(linear_msg);
    }

    if (receive == kfs_rz_) {
      auto linear_msg = r1_msgs::msg::LinearMotion();
      linear_msg.torque = msg->torque;
      linear_msg.speed = msg->speed;
      linear_msg.pos = msg->pos;
      kfs_rz_linear_motion_status_publisher_->publish(linear_msg);
    }

    if (receive == kfs_ryaw_) {
      auto angle_msg = r1_msgs::msg::AngleMotion();
      angle_msg.torque = msg->torque;
      angle_msg.speed = msg->speed;
      angle_msg.pos = msg->pos;
      kfs_ryaw_angle_motion_status_publisher_->publish(angle_msg);
    }

    if (receive == front_expand_) {
      auto linear_msg = r1_msgs::msg::LinearMotion();
      linear_msg.torque = msg->torque;
      linear_msg.speed = msg->speed;
      linear_msg.pos = msg->pos;
      front_expand_linear_motion_status_publisher_->publish(linear_msg);
    }

    if (receive == rear_expand_) {
      auto linear_msg = r1_msgs::msg::LinearMotion();
      linear_msg.torque = msg->torque;
      linear_msg.speed = msg->speed;
      linear_msg.pos = msg->pos;
      rear_expand_linear_motion_status_publisher_->publish(linear_msg);
    }

    if (receive == r2_lift_) {
      r2_lift_motor_status_publisher_->publish(msg_status);
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

    if (receive == kfs_front_switch_) {
      r1_msgs::msg::GpioInput gpio_msg;
      gpio_msg.status = msg->input;
      kfs_front_switch_status_publisher_->publish(gpio_msg);
    }
    if (receive == kfs_rear_switch_) {
      r1_msgs::msg::GpioInput gpio_msg;
      gpio_msg.status = msg->input;
      kfs_rear_switch_status_publisher_->publish(gpio_msg);
    }

    // デバッグ用パブリッシュ（GPIO入力）
    r1_msgs::msg::GpioInput gpio_msg;
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
    // RCLCPP_INFO(
    //   this->get_logger(), "wheel_speeds_ref: FL=%f, FR=%f, RL=%f, RR=%f", msg->fl_wheel_speed,
    //   msg->fr_wheel_speed, msg->rl_wheel_speed, msg->rr_wheel_speed);
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

      mecanum_single_ref_publisher_[i]->publish(msg_ref);
    }
  }

  // TODO: callbackのところはcallback関数をいじって、処理を共通化する

  void kfs_fx_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_fx_single_ref_publisher_->publish(msg_ref);
    RCLCPP_INFO(
      this->get_logger(), "kfs_fx_motor_ref_callback: control_type=%s, ref=%f",
      msg_ref.control_type.c_str(), msg_ref.ref);
    std::string topic_name = "sabacan_robomas_ref" + std::to_string(kfs_fx_.board_id) + "/motor" +
                             std::to_string(kfs_fx_.number);
    RCLCPP_INFO(this->get_logger(), "Published to topic: %s", topic_name.c_str());
  }

  void kfs_fz_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_fz_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_fyaw_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_fyaw_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_rx_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_rx_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_rz_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_rz_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_ryaw_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_ryaw_single_ref_publisher_->publish(msg_ref);
  }

  void front_expand_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    front_expand_single_ref_publisher_->publish(msg_ref);
  }

  void rear_expand_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    rear_expand_single_ref_publisher_->publish(msg_ref);
  }

  void r2_lift_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    r2_lift_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_front_pump_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = kfs_front_pump_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void kfs_rear_pump_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = kfs_rear_pump_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void kfs_front_valve_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = kfs_front_valve_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void kfs_rear_valve_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = kfs_rear_valve_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
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

  // ======== Sabacan Publisher and Subscription =========
  // gpio_refのpublisherを宣言
  std::vector<rclcpp::Publisher<sabacan_msgs::msg::SabacanGPIORefInt>::SharedPtr>
    sabacan_gpio_ref_int_publisher_;
  std::vector<rclcpp::Publisher<sabacan_msgs::msg::SabacanGPIORefFloat>::SharedPtr>
    sabacan_gpio_ref_float_publisher_;
  // robomas_statusのsubscriptionを宣言
  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr>
    sabacan_robomas_status_subscription_;
  // gpio_statusのsubscriptionを宣言
  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIOStatus>::SharedPtr>
    sabacan_gpio_status_subscription_;
  // ---------- sabacan_single_control_msgsのpublisherを宣言 ----------
  // メカナム
  std::vector<
    rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr>
    mecanum_single_ref_publisher_{MECANUM_NUM};
  // KFS回収機構
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_fx_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_fz_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_fyaw_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_rx_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_rz_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_ryaw_single_ref_publisher_;
  // 展開
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    front_expand_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    rear_expand_single_ref_publisher_;
  // R2昇降
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    r2_lift_single_ref_publisher_;

  // ======== R1 Machine Publisher and Subscription ========
  // -------- ロボマス ----------
  // メカナムの指令値
  rclcpp::Subscription<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_ref_subscription_;
  // メカナムホイールの回転速度のフィードバック
  rclcpp::Publisher<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_feedback_publisher_;
  // オドメトリのフィードバック
  rclcpp::Publisher<r1_msgs::msg::OdometryEncoder>::SharedPtr odometry_encoder_publisher_;
  // KFS回収機構の指令値
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_fx_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_fz_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_fyaw_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_rx_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_rz_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_ryaw_motor_ref_subscription_;
  // KFS回収機構のフィードバック
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr kfs_fx_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr kfs_fz_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::AngleMotion>::SharedPtr kfs_fyaw_angle_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr kfs_rx_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr kfs_rz_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::AngleMotion>::SharedPtr kfs_ryaw_angle_motion_status_publisher_;
  // 展開の指令値
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr front_expand_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr rear_expand_motor_ref_subscription_;
  // 展開のフィードバック
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr
    front_expand_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr
    rear_expand_linear_motion_status_publisher_;
  // R2昇降の指令値
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr r2_lift_motor_ref_subscription_;
  // R2昇降のフィードバック
  rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr r2_lift_motor_status_publisher_;
  // --------- GPIO ----------
  // KFS真空ポンプ
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    kfs_front_pump_gpio_pwm_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    kfs_rear_pump_gpio_pwm_ref_subscription_;
  // 真空電磁弁
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    kfs_front_valve_gpio_pwm_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    kfs_rear_valve_gpio_pwm_ref_subscription_;
  // KFSリミットスイッチ
  rclcpp::Publisher<r1_msgs::msg::GpioInput>::SharedPtr kfs_front_switch_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioInput>::SharedPtr kfs_rear_switch_status_publisher_;

  // ========= Board id ==========
  // ---------- ロボマス ----------
  std::vector<BoardInfo> mecanum_ = {
    {.board_id = 1, .number = 0},
    {.board_id = 1, .number = 1},
    {.board_id = 1, .number = 2},
    {.board_id = 1, .number = 3}};
  std::vector<BoardInfo> odometry_encoder_ = {
    {.board_id = 1, .number = 0}, {.board_id = 1, .number = 1}};
  BoardInfo kfs_fx_ = {.board_id = 2, .number = 0};
  BoardInfo kfs_fz_ = {.board_id = 2, .number = 1};
  BoardInfo kfs_fyaw_ = {.board_id = 2, .number = 2};
  BoardInfo front_expand_ = {.board_id = 2, .number = 3};
  BoardInfo kfs_rx_ = {.board_id = 3, .number = 0};
  BoardInfo kfs_rz_ = {.board_id = 3, .number = 1};
  BoardInfo kfs_ryaw_ = {.board_id = 3, .number = 2};
  BoardInfo rear_expand_ = {.board_id = 3, .number = 3};
  BoardInfo r2_lift_ = {.board_id = 4, .number = 0};
  // ---------- GPIO ----------
  BoardInfo kfs_front_pump_ = {.board_id = 1, .number = 0};
  BoardInfo kfs_rear_pump_ = {.board_id = 1, .number = 1};
  BoardInfo kfs_front_valve_ = {.board_id = 1, .number = 2};
  BoardInfo kfs_rear_valve_ = {.board_id = 1, .number = 3};
  BoardInfo kfs_front_switch_ = {.board_id = 2, .number = 0};
  BoardInfo kfs_rear_switch_ = {.board_id = 2, .number = 1};

  std::vector<double> odometry_encoder_pos_values_ = std::vector<double>(ODOMETRY_NUM);
  std::vector<double> odometry_encoder_speed_values_ = std::vector<double>(ODOMETRY_NUM);
  std::vector<double> mecanum_wheel_speeds_feedback_ = std::vector<double>(MECANUM_NUM);
  static constexpr int FL = 0;
  static constexpr int FR = 1;
  static constexpr int RL = 2;
  static constexpr int RR = 3;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int MECANUM_NUM = 4;
  static constexpr int ODOMETRY_NUM = 2;
  // デバッグ用
  std::vector<DebugMotorInfo> debug_motor_;
  std::vector<DebugGPIOInputInfo> debug_gpio_input_;
  std::vector<bool> odometry_encoder_update_ = std::vector<bool>(ODOMETRY_NUM, false);

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
