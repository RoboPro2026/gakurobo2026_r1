/**
 * @file r1_swerve_drive_node.cpp
 * @author Yamaguchi Yudai (yudai.yy0804@gmail.com)
 * @brief 4輪独立ステアリング機構のノード
 * @version 0.1
 * @date 2026-02-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <chrono>
#include <limits>

#include "geometry_msgs/msg/twist.hpp"
#include "r1_msgs/msg/swerve_drive.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// TODO:初期化処理を行う。
// TODO: ステアの状態を指定できるようにする。

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_swerve_drive_node")
  {
    // 速度指令値Subscription
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MyNode::cmd_vel_callback, this, std::placeholders::_1));

    // ホイールの速度指令値Publisher
    swerve_drive_ref_publisher_ =
      this->create_publisher<r1_msgs::msg::SwerveDrive>("/swerve_drive_ref", 10);

    parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    // BNO086以外のIMUを使う場合は、適宜変えること。
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/bno086/imu/data_raw", 10, std::bind(&MyNode::imu_callback, this, std::placeholders::_1));

    // yawのオフセット
    yaw_offset_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "yaw_offset", 10, std::bind(&MyNode::yaw_offset_callback, this, std::placeholders::_1));

    this->declare_parameter("robot_length", 0.5);
    this->declare_parameter("robot_width", 0.5);
    this->declare_parameter("wheel_radius", 0.1);
    this->declare_parameter("wheel_gear_ratio", 1.0);
    this->declare_parameter("steering_gear_ratio", 1.0);
    this->declare_parameter("wheel_speed_limit", 100.0);
    this->declare_parameter("steering_angle_limit", 6.28);
    this->declare_parameter("wheel_motor_inverse", std::vector<bool>{false, false, false, false});
    this->declare_parameter(
      "steering_motor_inverse", std::vector<bool>{false, false, false, false});
    this->declare_parameter("use_imu", true);

    this->get_parameter("robot_length", robot_length_);
    this->get_parameter("robot_width", robot_width_);
    robot_radius_ = std::sqrt(std::pow(robot_length_ / 2.0, 2) + std::pow(robot_width_ / 2.0, 2));
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("wheel_gear_ratio", wheel_gear_ratio_);
    this->get_parameter("steering_gear_ratio", steering_gear_ratio_);
    this->get_parameter("wheel_speed_limit", wheel_speed_limit_);
    this->get_parameter("steering_angle_limit", steering_angle_limit_);
    std::vector<bool> wheel_motor_inverse(4);
    this->get_parameter("wheel_motor_inverse", wheel_motor_inverse);
    for (size_t i = 0; i < 4; i++) {
      wheel_motor_dir_[i] = wheel_motor_inverse[i] ? -1.0 : 1.0;
    }
    std::vector<bool> steering_motor_inverse(4);
    this->get_parameter("steering_motor_inverse", steering_motor_inverse);
    for (size_t i = 0; i < 4; i++) {
      steering_motor_dir_[i] = steering_motor_inverse[i] ? -1.0 : 1.0;
    }
    this->get_parameter("use_imu", use_imu_);
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "robot_length") {
        robot_length_ = parameter.as_double();
        robot_radius_ =
          std::sqrt(std::pow(robot_length_ / 2.0, 2) + std::pow(robot_width_ / 2.0, 2));
        RCLCPP_INFO(this->get_logger(), "Updated parameter: robot_length = %.3f", robot_length_);
      } else if (name == "robot_width") {
        robot_width_ = parameter.as_double();
        robot_radius_ =
          std::sqrt(std::pow(robot_length_ / 2.0, 2) + std::pow(robot_width_ / 2.0, 2));
        RCLCPP_INFO(this->get_logger(), "Updated parameter: robot_width = %.3f", robot_width_);
      } else if (name == "wheel_radius") {
        wheel_radius_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: wheel_radius = %.3f", wheel_radius_);
      } else if (name == "wheel_gear_ratio") {
        wheel_gear_ratio_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: wheel_gear_ratio = %.3f", wheel_gear_ratio_);
      } else if (name == "steering_gear_ratio") {
        steering_gear_ratio_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: steering_gear_ratio = %.3f",
          steering_gear_ratio_);
      } else if (name == "wheel_speed_limit") {
        wheel_speed_limit_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: wheel_speed_limit = %.3f", wheel_speed_limit_);
      } else if (name == "steering_angle_limit") {
        steering_angle_limit_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: steering_angle_limit = %.3f",
          steering_angle_limit_);
      } else if (name == "wheel_motor_inverse") {
        std::vector<bool> wheel_motor_inverse = parameter.as_bool_array();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: wheel_motor_inverse = [%s, %s, %s, %s]",
          wheel_motor_inverse[0] ? "true" : "false", wheel_motor_inverse[1] ? "true" : "false",
          wheel_motor_inverse[2] ? "true" : "false", wheel_motor_inverse[3] ? "true" : "false");
        for (int i = 0; i < N; i++) {
          wheel_motor_dir_[i] = wheel_motor_inverse[i] ? -1.0 : 1.0;
        }
      } else if (name == "steering_motor_inverse") {
        std::vector<bool> steering_motor_inverse = parameter.as_bool_array();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: steering_motor_inverse = [%s, %s, %s, %s]",
          steering_motor_inverse[0] ? "true" : "false",
          steering_motor_inverse[1] ? "true" : "false",
          steering_motor_inverse[2] ? "true" : "false",
          steering_motor_inverse[3] ? "true" : "false");
        for (int i = 0; i < N; i++) {
          steering_motor_dir_[i] = steering_motor_inverse[i] ? -1.0 : 1.0;
        }
      } else if (name == "use_imu") {
        use_imu_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: use_imu = %s", use_imu_ ? "true" : "false");
      } else {
        result.successful = false;
        result.reason = "Invalid parameter name: " + name;
        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
      }
    }
    return result;
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double vx_ref = msg->linear.x;
    double vy_ref = msg->linear.y;
    double omega_ref = msg->angular.z;

    calculate_swerve_drive(vx_ref, vy_ref, omega_ref, yaw_);
    swerve_drive_ref_publisher_->publish(swerve_drive_ref_);
    RCLCPP_INFO(
      this->get_logger(), "Received cmd_vel: vx=%.3f, vy=%.3f, omega=%.3f", vx_ref, vy_ref,
      omega_ref);
    RCLCPP_INFO(
      this->get_logger(),
      "Calculated swerve drive ref: v0=%.3f, theta0=%.3f, v1=%.3f, "
      "theta1=%.3f, v2=%.3f, theta2=%.3f, v3=%.3f, theta3=%.3f",
      swerve_drive_ref_.v0, swerve_drive_ref_.theta0, swerve_drive_ref_.v1,
      swerve_drive_ref_.theta1, swerve_drive_ref_.v2, swerve_drive_ref_.theta2,
      swerve_drive_ref_.v3, swerve_drive_ref_.theta3);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // imuを使わない設定の場合は処理しない
    if (!use_imu_) return;

    // IMUのyaw角の情報を更新
    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double yaw, pitch, roll;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_ = yaw + yaw_offset_;
    // RCLCPP_INFO(this->get_logger(), "yaw = %f", yaw_);
  }

  void yaw_offset_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // imuを使わない設定の場合は処理しない
    if (!use_imu_) return;

    yaw_offset_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "yaw_offset = %f", yaw_offset_);
  }

  void calculate_swerve_drive(double vx_ref, double vy_ref, double omega_ref, double theta)
  {
    double wheel_vx[4];
    double wheel_vy[4];
    double wheel_v[4];
    double steer_theta[4];
    double R = robot_radius_;

    // 座標系
    // y
    // ↑
    // |
    // |
    // O----------→ x

    // 4輪独立ステアリングの逆運動学の計算
    for (int i = 0; i < 4; i++) {
      wheel_vx[i] = vx_ref - R * omega_ref * std::sin(theta + i * M_PI / 2.0);
      wheel_vy[i] = vy_ref + R * omega_ref * std::cos(theta + i * M_PI / 2.0);
      wheel_v[i] = std::sqrt(std::pow(wheel_vx[i], 2) + std::pow(wheel_vy[i], 2));
      // TODO: thetaが連続となるようにする。
      steer_theta[i] = std::atan2(wheel_vy[i], wheel_vx[i]);
    }

    // 計算した回転速度がlimitより高いかを確認
    double max_speed = std::abs(wheel_v[0]);
    for (int i = 1; i < 4; i++) {
      max_speed = std::max(max_speed, std::abs(wheel_v[i]));
    }
    if (max_speed > wheel_speed_limit_) {
      double scale = wheel_speed_limit_ / max_speed;
      swerve_drive_ref_.v0 *= scale;
      swerve_drive_ref_.v1 *= scale;
      swerve_drive_ref_.v2 *= scale;
      swerve_drive_ref_.v3 *= scale;
      RCLCPP_ERROR(
        this->get_logger(), "Wheel speed limit exceeded. Scaling down by factor %.3f", scale);
    }

    // 計算したステアリング角度がlimitより高いかを確認
    // ステアリングの角度制限を超えたらクランプしてしまうと、それはそれで危ないので、ERRORを出すだけにする。
    for (int i = 0; i < 4; i++) {
      if (std::abs(steer_theta[i]) > steering_angle_limit_) {
        RCLCPP_ERROR(
          this->get_logger(), "Steering angle limit exceeded for wheel %d: %.3f rad", i,
          steer_theta[i]);
      }
    }

    // 回転方向とギア比を考慮し代入
    swerve_drive_ref_.v0 = wheel_v[0] * wheel_motor_dir_[0] / wheel_gear_ratio_;
    swerve_drive_ref_.v1 = wheel_v[1] * wheel_motor_dir_[1] / wheel_gear_ratio_;
    swerve_drive_ref_.v2 = wheel_v[2] * wheel_motor_dir_[2] / wheel_gear_ratio_;
    swerve_drive_ref_.v3 = wheel_v[3] * wheel_motor_dir_[3] / wheel_gear_ratio_;
    swerve_drive_ref_.theta0 = steer_theta[0] * steering_motor_dir_[0] / steering_gear_ratio_;
    swerve_drive_ref_.theta1 = steer_theta[1] * steering_motor_dir_[1] / steering_gear_ratio_;
    swerve_drive_ref_.theta2 = steer_theta[2] * steering_motor_dir_[2] / steering_gear_ratio_;
    swerve_drive_ref_.theta3 = steer_theta[3] * steering_motor_dir_[3] / steering_gear_ratio_;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<r1_msgs::msg::SwerveDrive>::SharedPtr swerve_drive_ref_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_offset_subscription_;
  r1_msgs::msg::SwerveDrive swerve_drive_ref_;

  // ========== 機械パラメータ ==========
  static constexpr int N = 4;
  double robot_length_;         // ロボットの長さ (m)
  double robot_width_;          // ロボットの幅 (m)
  double robot_radius_;         // ロボットの半径 (m) = sqrt((length/2)^2 + (width/2)^2)
  double wheel_radius_;         // ホイールの半径 (m)
  double wheel_gear_ratio_;     // ホイールのギア比（減速比）。ギア比で割った値が出力される
  double steering_gear_ratio_;  // ステアリングのギア比（減速比）。ギア比で割った値が出力される
  // ========== 制御パラメータ ==========
  double wheel_speed_limit_;     // ホイールの速度制限 (rad/s)
  double steering_angle_limit_;  // ステアリング角度の制限 (rad)
  // motor_inverse = trueのとき、motor_dir_が-1.0になる。
  std::vector<double> wheel_motor_dir_ = {1.0, 1.0, 1.0, 1.0};
  std::vector<double> steering_motor_dir_ = {1.0, 1.0, 1.0, 1.0};
  bool use_imu_;       // IMUを使用するかどうか
  double yaw_offset_;  // IMUのyaw角のオフセット (rad)
  double yaw_;         // IMUのyaw角
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}