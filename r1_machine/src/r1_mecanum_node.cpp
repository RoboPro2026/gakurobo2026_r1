/**
 * @file r1_mecanum_node.cpp
 * @author Yamaguchi Yudai
 * @brief メカナムホイールの逆運動学を計算するノード
 * @version 0.1
 * @date 2025-09-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// clang-format off
  //    FL(0) ---- FR(1)
  //      |          |
  //      |   ロボット  |
  //      |          |
  //    RL(2) ---- RR(3)

  // FL: Front Left   FR: Front Right
  // RL: Rear Left    RR: Rear Right
  // 全モータが正回転すると、x方向に進むように定義。
  // 下のアスキーアートのときをロボットが0度のときと定義。
  //
  //                 y
  //                 ^
  //                 |
  //                 |
  //  FL(0) ↙︎       |      ↖︎ FR(1)
  //        /------------------\
  //        |        |         |
  //        |        |         |
  //        |                  | 
  //        |        |         |
  // <---x--|--------O---------|
  //        |       ↻ w       |
  //        |                  | 
  //        |                  |
  //        |                  | 
  //        \------------------/
  //  RL(2) ↖︎                  ↙︎ RR(3)
// clang-format on

#include <chrono>
#include <limits>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_mecanum_node")
  {
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MyNode::cmd_vel_callback, this, std::placeholders::_1));

    wheel_speeds_publisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/mecanum_wheel_speeds", 10);

    parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    auto parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    // パラメータの範囲を設定
    auto range = rcl_interfaces::msg::FloatingPointRange();
    // 入力は0より大きい値のみ（1e-9以上）
    range.from_value = 1e-9;
    range.to_value = std::numeric_limits<double>::max();
    // 刻み幅に制限なし
    range.step = 0.0;

    parameter_descriptor.floating_point_range.push_back(range);

    this->declare_parameter("wheel_radius", 0.1, parameter_descriptor);
    this->declare_parameter("robot_length", 0.5, parameter_descriptor);
    this->declare_parameter("robot_width", 0.25, parameter_descriptor);
    this->declare_parameter("speed_limit", 100.0, parameter_descriptor);

    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("robot_length", robot_length_);
    this->get_parameter("robot_width", robot_width_);
    this->get_parameter("speed_limit", speed_limit_);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    target_vel_ = *msg;
    // メカナムホイールの逆運動学を計算
    calculate_wheel_speeds(
      target_vel_.linear.x, target_vel_.linear.y, target_vel_.angular.z, theta_);

    // 角速度をFloat64MultiArrayでパブリッシュ
    auto wheel_speeds_msg = std_msgs::msg::Float64MultiArray();
    wheel_speeds_msg.data = {
      wheel_speeds_[FL], wheel_speeds_[FR], wheel_speeds_[RL], wheel_speeds_[RR]};
    wheel_speeds_publisher_->publish(wheel_speeds_msg);

    // デバッグ用
    RCLCPP_INFO(
      this->get_logger(), "Cmd Vel: x: %.2f, y: %.2f, omega: %.2f", target_vel_.linear.x,
      target_vel_.linear.y, target_vel_.angular.z);
    RCLCPP_INFO(
      this->get_logger(), "Wheel Speeds: FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f", wheel_speeds_[FL],
      wheel_speeds_[FR], wheel_speeds_[RL], wheel_speeds_[RR]);
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "wheel_radius") {
        wheel_radius_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: wheel_radius = %.3f", wheel_radius_);
      } else if (name == "robot_length") {
        robot_length_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: robot_length = %.3f", robot_length_);
      } else if (name == "robot_width") {
        robot_width_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: robot_width = %.3f", robot_width_);
      } else if (name == "speed_limit") {
        speed_limit_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: speed_limit = %.3f", speed_limit_);
      } else {
        result.successful = false;
        result.reason = "Invalid parameter name: " + name;
        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
      }
    }

    return result;
  }

  /**
   * @brief メカナムホイールの逆運動学を計算する
   * 
   * @param _vx xの速度[rad/s]
   * @param _vy yの速度[rad/s]
   * @param omega 角速度[rad/s]
   * @param theta ロボットの角度[rad]。y軸正方向を0度とし、反時計回りを正とする。IMUを使用しない場合はtheta=0
   */
  void calculate_wheel_speeds(double _vx, double _vy, double _omega, double theta)
  {
    double vx, vy, omega, max_speed;
    double L = robot_length_;
    double W = robot_width_;
    double R = wheel_radius_;

    // 回転行列を計算
    vx = _vx * cos(theta) + _vy * sin(theta);
    vy = -_vx * sin(theta) + _vy * cos(theta);
    omega = _omega;

    // メカナムホイールの逆運動学の計算
    wheel_speeds_[FL] = (1 / R) * (vx - vy + (L + W) * omega);
    wheel_speeds_[FR] = (1 / R) * (vx + vy + (L + W) * omega);
    wheel_speeds_[RL] = (1 / R) * (vx + vy - (L + W) * omega);
    wheel_speeds_[RR] = (1 / R) * (vx - vy - (L + W) * omega);

    // 計算した値がlimitより高いかを確認
    max_speed = std::abs(wheel_speeds_[FL]);
    for (int i = 1; i < 4; i++) {
      max_speed = std::max(max_speed, std::abs(wheel_speeds_[i]));
    }

    // 計算した値がlimitを超えていたら、limitに収まるように全体をスケーリング
    if (max_speed > speed_limit_) {
      for (int i = 0; i < 4; i++) {
        wheel_speeds_[i] = wheel_speeds_[i] / max_speed * speed_limit_;
      }
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speeds_publisher_;
  // 速度指令値
  geometry_msgs::msg::Twist target_vel_;
  double theta_ = 0.0;
  double speed_limit_;                             //rad/s
  double robot_length_;                            // ロボットの長さ (m)
  double robot_width_;                             // ロボットの幅 (m)
  double wheel_radius_;                            // ホイールの半径 (m)
  double wheel_speeds_[4] = {0.0, 0.0, 0.0, 0.0};  // FL, FR, RL, RR
  static constexpr int FL = 0;
  static constexpr int FR = 1;
  static constexpr int RL = 2;
  static constexpr int RR = 3;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}