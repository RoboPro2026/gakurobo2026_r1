/**
 * @file r1_angle_motion_node.cpp
 * @author Yamaguchi Yudai
 * @brief 回転軸用の原点検出付きモーション制御ノード
 * @version 0.1
 * @date 2025-11-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <chrono>
#include <cmath>

#include "r1_msgs/msg/angle_motion.hpp"
#include "r1_msgs/msg/gpio_input.hpp"
#include "r1_msgs/msg/motor_ref.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_angle_motion_node")
  {
    angle_motion_status_subscription_ = this->create_subscription<r1_msgs::msg::AngleMotion>(
      "/angle_motion_status", 10,
      std::bind(&MyNode::angle_motion_status_callback, this, std::placeholders::_1));

    low_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
      "/low_switch_status", 10,
      std::bind(&MyNode::low_switch_status_callback, this, std::placeholders::_1));

    high_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
      "/high_switch_status", 10,
      std::bind(&MyNode::high_switch_status_callback, this, std::placeholders::_1));

    angle_motion_ref_publisher_ =
      this->create_publisher<r1_msgs::msg::MotorRef>("/angle_motion_motor_ref", 10);

    position_ref_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/angle_motion_position_ref", 10,
      std::bind(&MyNode::position_ref_callback, this, std::placeholders::_1));

    detect_origin_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/angle_motion_detect_origin", 10,
      std::bind(&MyNode::detect_origin_callback, this, std::placeholders::_1));

    mode_status_publisher_ =
      this->create_publisher<std_msgs::msg::Int32>("/angle_motion_mode_status", 10);

    parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));

    // パラメータ宣言
    this->declare_parameter("use_low_switch", true);
    this->declare_parameter("use_high_switch", true);
    this->declare_parameter("torque_threshold", 1.0);              // Nm
    this->declare_parameter("origin_detect_threshold_time", 0.1);  // s
    // 原点検出時の速度は負の符号でも可、原点としたい方向に回転させる
    this->declare_parameter("origin_detect_speed", -3.14);  // rad/s
    this->declare_parameter("angle_min", -3.14);            // rad
    this->declare_parameter("angle_max", 3.14);             // rad
    this->declare_parameter("gear_ratio", 0.05);
    this->declare_parameter("inverse_motor", false);
    this->declare_parameter("inverse_low_switch_logic", false);
    this->declare_parameter("inverse_high_switch_logic", false);

    // パラメータ取得
    this->get_parameter("use_low_switch", use_low_switch_);
    this->get_parameter("use_high_switch", use_high_switch_);
    this->get_parameter("torque_threshold", torque_threshold_);
    this->get_parameter("origin_detect_threshold_time", origin_detect_threshold_time_);
    this->get_parameter("origin_detect_speed", origin_detect_speed_);
    this->get_parameter("angle_min", angle_min_);
    this->get_parameter("angle_max", angle_max_);
    this->get_parameter("gear_ratio", gear_ratio_);
    bool inverse_motor;
    this->get_parameter("inverse_motor", inverse_motor);
    motor_dir_ = inverse_motor ? -1.0 : 1.0;
    this->get_parameter("inverse_low_switch_logic", inverse_low_switch_logic_);
    this->get_parameter("inverse_high_switch_logic", inverse_high_switch_logic_);
  }

private:
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "use_low_switch") {
        use_low_switch_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: use_low_switch = %s",
          use_low_switch_ ? "true" : "false");
      } else if (name == "use_high_switch") {
        use_high_switch_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: use_high_switch = %s",
          use_high_switch_ ? "true" : "false");
      } else if (name == "torque_threshold") {
        torque_threshold_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: torque_threshold = %.3f", torque_threshold_);
      } else if (name == "origin_detect_threshold_time") {
        origin_detect_threshold_time_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: origin_detect_threshold_time = %.3f",
          origin_detect_threshold_time_);
      } else if (name == "origin_detect_speed") {
        origin_detect_speed_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: origin_detect_speed = %.3f",
          origin_detect_speed_);
      } else if (name == "angle_min") {
        angle_min_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: angle_min = %.3f", angle_min_);
      } else if (name == "angle_max") {
        angle_max_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: angle_max = %.3f", angle_max_);
      } else if (name == "gear_ratio") {
        gear_ratio_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: gear_ratio = %.3f", gear_ratio_);
      } else if (name == "inverse_motor") {
        bool inverse_motor = parameter.as_bool();
        motor_dir_ = inverse_motor ? -1.0 : 1.0;
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: inverse_motor = %s",
          inverse_motor ? "true" : "false");
      } else if (name == "inverse_low_switch_logic") {
        inverse_low_switch_logic_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: inverse_low_switch_logic = %s",
          inverse_low_switch_logic_ ? "true" : "false");
      } else if (name == "inverse_high_switch_logic") {
        inverse_high_switch_logic_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: inverse_high_switch_logic = %s",
          inverse_high_switch_logic_ ? "true" : "false");
      } else {
        result.successful = false;
        result.reason = "Invalid parameter name: " + name;
        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
      }
    }

    return result;
  }

  void angle_motion_status_callback(const r1_msgs::msg::AngleMotion::SharedPtr msg)
  {
    current_torque_ = msg->torque;
    current_speed_ = msg->speed;
    current_angle_ = msg->pos;
  }

  void low_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
  {
    low_switch_ = msg->status ^ inverse_low_switch_logic_;
  }

  void high_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
  {
    high_switch_ = msg->status ^ inverse_high_switch_logic_;
  }

  void position_ref_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double target_angle;

    if (mode_ == MODE_SPEED) {
      RCLCPP_ERROR(this->get_logger(), "Currently in speed mode, position ref ignored.");
      return;
    }

    // 範囲内に収める
    if (msg->data < angle_min_) {
      target_angle = angle_min_ + angle_offset_;
      RCLCPP_WARN(this->get_logger(), "Target angle below minimum. Clamping to %.3f", target_angle);
    } else if (msg->data > angle_max_) {
      target_angle = angle_max_ + angle_offset_;
      RCLCPP_WARN(this->get_logger(), "Target angle above maximum. Clamping to %.3f", target_angle);
    } else {
      target_angle = msg->data + angle_offset_;
    }

    auto motor_msg = r1_msgs::msg::MotorRef();
    motor_msg.control_type = "POSITION";
    double motor_angle = target_angle / gear_ratio_;
    motor_msg.ref = motor_dir_ * motor_angle;
    angle_motion_ref_publisher_->publish(motor_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing motor angle ref: %.3f", motor_msg.ref);
  }

  void detect_origin_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      mode_ = MODE_SPEED;
      // 最後に通常のトルクを検出した時刻を更新
      last_normal_torque_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Switched to speed control mode for origin detection.");
    } else {
      mode_ = MODE_POSITION;
      RCLCPP_INFO(this->get_logger(), "Switched to position control mode.");
    }
  }

  void timer_callback()
  {
    if (mode_ == MODE_SPEED) {
      bool detect_origin = false;

      // リミットスイッチが反応している場合
      detect_origin |= (use_low_switch_ && low_switch_);
      detect_origin |= (use_high_switch_ && high_switch_);

      // トルク制限を超えた場合（一定時間トルクのしきい値を超えた場合）
      if (std::abs(current_torque_) <= torque_threshold_) {
        // 通常のトルクが検出されたので、最後に通常のトルクを検出した時刻を更新
        last_normal_torque_time_ = this->now();
      }
      detect_origin |=
        ((this->now() - last_normal_torque_time_).seconds() > origin_detect_threshold_time_);

      auto motor_ref_msg = r1_msgs::msg::MotorRef();
      if (detect_origin) {
        // 原点検出時: オフセットを更新し、位置制御モードへ切り替え
        mode_ = MODE_POSITION;
        angle_offset_ = gear_ratio_ * current_angle_;
        motor_ref_msg.control_type = "POSITION";
        motor_ref_msg.ref = current_angle_;
        RCLCPP_INFO(this->get_logger(), "Origin detected at angle: %.3f", angle_offset_);
      } else {
        // 原点検出中は負の方向にモータを回転させる
        motor_ref_msg.control_type = "VELOCITY";
        motor_ref_msg.ref = motor_dir_ * origin_detect_speed_;
      }
      angle_motion_ref_publisher_->publish(motor_ref_msg);
    }
    // モードをPublish
    auto mode_msg = std_msgs::msg::Int32();
    mode_msg.data = mode_;
    mode_status_publisher_->publish(mode_msg);
  }

  rclcpp::Subscription<r1_msgs::msg::AngleMotion>::SharedPtr angle_motion_status_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr low_switch_status_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr high_switch_status_subscription_;
  rclcpp::Publisher<r1_msgs::msg::MotorRef>::SharedPtr angle_motion_ref_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_ref_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detect_origin_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_status_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_normal_torque_time_ = rclcpp::Time(0);
  bool use_low_switch_ = true;
  bool use_high_switch_ = true;
  double origin_detect_speed_ = 0.0;
  double torque_threshold_ = 0.0;
  double origin_detect_threshold_time_ = 0.0;
  double motor_dir_ = 1.0;
  double angle_min_ = -3.14;
  double angle_max_ = 3.14;
  // オフセット補正用
  double angle_offset_ = 0.0;
  bool low_switch_ = false;
  bool high_switch_ = false;
  bool inverse_low_switch_logic_ = false;
  bool inverse_high_switch_logic_ = false;
  double current_torque_ = 0.0;
  double current_speed_ = 0.0;
  double current_angle_ = 0.0;
  // 減速比。出力角度は計算値を `gear_ratio` で割った後に配信される
  double gear_ratio_ = 0.05;
  static constexpr int MODE_POSITION = 0;
  static constexpr int MODE_SPEED = 1;
  int mode_ = MODE_POSITION;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
