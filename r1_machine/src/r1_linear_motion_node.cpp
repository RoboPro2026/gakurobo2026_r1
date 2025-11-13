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
#include "r1_msgs/msg/motor.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_linear_motion_node")
  {
    linear_motion_status_subscription_ = this->create_subscription<r1_msgs::msg::LinearMotion>(
      "/linear_motion_status", 10,
      std::bind(&MyNode::linear_motion_status_callback, this, std::placeholders::_1));

    motor_position_ref_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/linear_motion_motor_position_ref", 10);

    position_ref_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/linear_motion_positon_ref", 10,
      std::bind(&MyNode::positon_ref_callback, this, std::placeholders::_1));

    parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    this->declare_parameter("use_low_switch", true);
    this->declare_parameter("use_high_switch", true);
    this->declare_parameter("pos_min", 0.0);
    this->declare_parameter("pos_max", 1.0);
    this->declare_parameter("pos_offset", 0.0);
    this->declare_parameter("inverse_motor", false);

    this->get_parameter("use_low_switch", use_low_switch_);
    this->get_parameter("use_high_switch", use_high_switch_);
    this->get_parameter("pos_min", pos_min_);
    this->get_parameter("pos_max", pos_max_);
    this->get_parameter("pos_offset", pos_offset_);
    bool inverse_motor;
    this->get_parameter("inverse_motor", inverse_motor);
    motor_dir_ = inverse_motor ? -1.0 : 1.0;
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
      } else if (name == "pos_min") {
        pos_min_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: pos_min = %.3f", pos_min_);
      } else if (name == "pos_max") {
        pos_max_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: pos_max = %.3f", pos_max_);
      } else if (name == "pos_offset") {
        pos_offset_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: pos_offset = %.3f", pos_offset_);
      } else if (name == "inverse_motor") {
        bool inverse_motor = parameter.as_bool();
        motor_dir_ = inverse_motor ? -1.0 : 1.0;
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: inverse_motor = %s",
          inverse_motor ? "true" : "false");
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
    low_switch_ = msg->low_switch;
    high_switch_ = msg->high_switch;
    current_pos_ = msg->pos;
  }

  // TODO: 必要であれば、モータがストールしたときは停止する機能を追加する
  void positon_ref_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double target_pos = msg->data + pos_offset_;

    // TODO: リミットスイッチが反応したときの処理を考える
    // リミットスイッチの状態を確認
    if (use_low_switch_ && low_switch_ && target_pos < current_pos_) {
      RCLCPP_WARN(this->get_logger(), "Low limit switch is activated. Cannot move lower.");
      return;
    }
    if (use_high_switch_ && high_switch_ && target_pos > current_pos_) {
      RCLCPP_WARN(this->get_logger(), "High limit switch is activated. Cannot move higher.");
      return;
    }

    // 範囲内に収める
    if (target_pos < pos_min_) {
      target_pos = pos_min_;
      RCLCPP_WARN(this->get_logger(), "Target position below minimum. Clamping to %.3f", pos_min_);
    } else if (target_pos > pos_max_) {
      target_pos = pos_max_;
      RCLCPP_WARN(this->get_logger(), "Target position above maximum. Clamping to %.3f", pos_max_);
    }

    auto motor_msg = std_msgs::msg::Float64();
    motor_msg.data = target_pos * motor_dir_;
    motor_position_ref_publisher_->publish(motor_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing motor position ref: %.3f", motor_msg.data);
  }

private:
  rclcpp::Subscription<r1_msgs::msg::LinearMotion>::SharedPtr linear_motion_status_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_position_ref_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_ref_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;
  bool use_low_switch_ = true;
  bool use_high_switch_ = true;
  double motor_dir_ = 1.0;
  double pos_min_ = 0.0;
  double pos_max_ = 1.0;
  // オフセット補正用
  double pos_offset_ = 0.0;
  bool low_switch_ = false;
  bool high_switch_ = false;
  double current_pos_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
