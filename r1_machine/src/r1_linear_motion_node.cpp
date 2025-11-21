/**
 * @file r1_linear_motion_node.cpp
 * @author Yamaguchi Yudai
 * @brief 
 * @version 0.1
 * @date 2025-11-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <chrono>

#include "r1_msgs/msg/limit_switch.hpp"
#include "r1_msgs/msg/linear_motion.hpp"
#include "r1_msgs/msg/motor_ref.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_linear_motion_node")
  {
    linear_motion_status_subscription_ = this->create_subscription<r1_msgs::msg::LinearMotion>(
      "/linear_motion_status", 10,
      std::bind(&MyNode::linear_motion_status_callback, this, std::placeholders::_1));

    linear_motion_ref_publisher_ =
      this->create_publisher<r1_msgs::msg::MotorRef>("/linear_motion_motor_ref", 10);

    position_ref_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/linear_motion_positon_ref", 10,
      std::bind(&MyNode::positon_ref_callback, this, std::placeholders::_1));

    detect_origin_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/linear_motion_detect_origin", 10,
      std::bind(&MyNode::detect_origin_callback, this, std::placeholders::_1));

    parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));

    this->declare_parameter("use_low_switch", true);
    this->declare_parameter("use_high_switch", true);
    this->declare_parameter("torque_threshold", 1.0);              // Nm
    this->declare_parameter("origin_detect_threshold_time", 0.1);  // s
    this->declare_parameter("origin_detect_speed", 3.14);          // rad
    this->declare_parameter("pos_min", 0.0);                       // m
    this->declare_parameter("pos_max", 1.0);                       // m
    this->declare_parameter("radius", 0.05);                       // m
    this->declare_parameter("inverse_motor", false);
    this->declare_parameter("inverse_low_switch_logic", false);
    this->declare_parameter("inverse_high_switch_logic", false);

    this->get_parameter("use_low_switch", use_low_switch_);
    this->get_parameter("use_high_switch", use_high_switch_);
    this->get_parameter("torque_threshold", torque_threshold_);
    this->get_parameter("origin_detect_threshold_time", origin_detect_threshold_time_);
    this->get_parameter("origin_detect_speed", origin_detect_speed_);
    this->get_parameter("pos_min", pos_min_);
    this->get_parameter("pos_max", pos_max_);
    this->get_parameter("radius", radius_);
    bool inverse_motor;
    this->get_parameter("inverse_motor", inverse_motor);
    motor_dir_ = inverse_motor ? -1.0 : 1.0;
    this->get_parameter("inverse_low_switch_logic", inverse_low_switch_logic_);
    this->get_parameter("inverse_high_switch_logic", inverse_high_switch_logic_);
  }

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
      } else if (name == "pos_min") {
        pos_min_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: pos_min = %.3f", pos_min_);
      } else if (name == "pos_max") {
        pos_max_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: pos_max = %.3f", pos_max_);
      } else if (name == "radius") {
        radius_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: radius = %.3f", radius_);
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

  void linear_motion_status_callback(const r1_msgs::msg::LinearMotion::SharedPtr msg)
  {
    low_switch_ = msg->low_switch ^ inverse_low_switch_logic_;
    high_switch_ = msg->high_switch ^ inverse_high_switch_logic_;
    current_torque_ = msg->torque;
    current_speed_ = msg->speed;
    current_pos_ = msg->pos;
  }

  void positon_ref_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // TODO: 気が向いたら、リミットスイッチが反応したときの動作を追加する
    double target_pos;

    if (mode_ == MODE_SPEED) {
      RCLCPP_ERROR(this->get_logger(), "Currently in speed mode, position ref ignored.");
      return;
    }

    // 範囲内に収める
    if (msg->data < pos_min_) {
      target_pos = pos_min_ + pos_offset_;
      RCLCPP_WARN(
        this->get_logger(), "Target position below minimum. Clamping to %.3f", target_pos);
    } else if (msg->data > pos_max_) {
      target_pos = pos_max_ + pos_offset_;
      RCLCPP_WARN(
        this->get_logger(), "Target position above maximum. Clamping to %.3f", target_pos);
    } else {
      target_pos = msg->data + pos_offset_;
    }

    auto motor_msg = r1_msgs::msg::MotorRef();
    motor_msg.control_type = "POSITION";
    double target_angle = target_pos / radius_;
    motor_msg.ref = motor_dir_ * target_angle;
    linear_motion_ref_publisher_->publish(motor_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing motor position ref: %.3f", motor_msg.ref);
  }

  void detect_origin_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      mode_ = MODE_SPEED;
      // 最後に通常のトルクを検出した時刻を更新
      last_normal_torque_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Switched to speed control mode.");
    } else {
      mode_ = MODE_POSITON;
      RCLCPP_INFO(this->get_logger(), "Switched to position control mode.");
    }
  }

  void timer_callback()
  {
    if (mode_ == MODE_SPEED) {
      bool detect_origin = false;
      // リミットスイッチが反応している場合
      // TODO: 必要であれば、リミットスイッチの反応時間のしきい値を設ける
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
        // 原点検出時
        // オフセットを更新し、位置制御モードへ切り替え
        mode_ = MODE_POSITON;
        pos_offset_ = radius_ * current_pos_;
        motor_ref_msg.control_type = "POSITION";
        motor_ref_msg.ref = current_pos_;
        RCLCPP_INFO(this->get_logger(), "Origin detected at position: %.3f", pos_offset_);
      } else {
        // 原研検出時は負の方向にモータを回転させる。
        motor_ref_msg.control_type = "VELOCITY";
        motor_ref_msg.ref = -motor_dir_ * origin_detect_speed_;
      }
      linear_motion_ref_publisher_->publish(motor_ref_msg);
      // RCLCPP_INFO(this->get_logger(), "Publishing motor speed ref: %.3f", motor_ref_msg.ref);
    }
  }

private:
  rclcpp::Subscription<r1_msgs::msg::LinearMotion>::SharedPtr linear_motion_status_subscription_;
  rclcpp::Publisher<r1_msgs::msg::MotorRef>::SharedPtr linear_motion_ref_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_ref_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detect_origin_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time detect_time_start_ = rclcpp::Time(0);
  rclcpp::Time last_normal_torque_time_ = rclcpp::Time(0);
  bool use_low_switch_ = true;
  bool use_high_switch_ = true;
  double origin_detect_speed_ = 0.0;
  double torque_threshold_ = 0.0;
  double origin_detect_threshold_time_ = 0.0;
  double motor_dir_ = 1.0;
  double pos_min_ = 0.0;
  double pos_max_ = 1.0;
  // オフセット補正用
  double pos_offset_ = 0.0;
  bool low_switch_ = false;
  bool high_switch_ = false;
  bool inverse_low_switch_logic_ = false;
  bool inverse_high_switch_logic_ = false;
  double current_torque_ = 0.0;
  double current_speed_ = 0.0;
  double current_pos_ = 0.0;
  double radius_ = 0.05;  // m、値は適当
  static constexpr int MODE_POSITON = 0;
  static constexpr int MODE_SPEED = 1;
  int mode_ = MODE_POSITON;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
