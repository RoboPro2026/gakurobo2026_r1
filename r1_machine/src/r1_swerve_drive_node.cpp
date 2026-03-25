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
#include <complex>
#include <limits>

#include "geometry_msgs/msg/twist.hpp"
#include "r1_msgs/msg/swerve_drive.hpp"
#include "r1_util/r1_util.h"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

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

    // 手動でswerve_drive_refを指定するためのSubscription
    manual_swerve_drive_ref_subscription_ = this->create_subscription<r1_msgs::msg::SwerveDrive>(
      "/manual_swerve_drive_ref", 10,
      std::bind(&MyNode::manual_swerve_drive_ref_callback, this, std::placeholders::_1));

    parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    // BNO086以外のIMUを使う場合は、適宜変えること。
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/bno086/imu/data_raw", 10, std::bind(&MyNode::imu_callback, this, std::placeholders::_1));

    // yawのオフセット
    set_swerve_drive_yaw_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "set_swerve_drive_yaw", 10,
      std::bind(&MyNode::set_swerve_drive_yaw_callback, this, std::placeholders::_1));

    this->declare_parameter("robot_length", 0.5);
    this->declare_parameter("robot_width", 0.5);
    this->declare_parameter("wheel_radius", 0.1);
    this->declare_parameter("wheel_gear_ratio", 1.0);
    this->declare_parameter("steer_gear_ratio", 1.0);
    this->declare_parameter("wheel_speed_limit", 100.0);
    this->declare_parameter("steer_angle_limit", 6.28);
    this->declare_parameter("angle_diff_range", 0.5);
    this->declare_parameter("steer_theta_offset", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("wheel_motor_inverse", std::vector<bool>{false, false, false, false});
    this->declare_parameter("steer_motor_inverse", std::vector<bool>{false, false, false, false});
    this->declare_parameter("use_imu", true);

    this->get_parameter("robot_length", robot_length_);
    this->get_parameter("robot_width", robot_width_);
    robot_radius_ = std::sqrt(std::pow(robot_length_ / 2.0, 2) + std::pow(robot_width_ / 2.0, 2));
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("wheel_gear_ratio", wheel_gear_ratio_);
    this->get_parameter("steer_gear_ratio", steer_gear_ratio_);
    this->get_parameter("wheel_speed_limit", wheel_speed_limit_);
    this->get_parameter("steer_angle_limit", steer_angle_limit_);
    this->get_parameter("angle_diff_range", angle_diff_range_);
    this->get_parameter("steer_theta_offset", steer_theta_offset_);
    std::vector<bool> wheel_motor_inverse(4);
    this->get_parameter("wheel_motor_inverse", wheel_motor_inverse);
    for (size_t i = 0; i < 4; i++) {
      wheel_motor_dir_[i] = wheel_motor_inverse[i] ? -1.0 : 1.0;
    }
    std::vector<bool> steer_motor_inverse(4);
    this->get_parameter("steer_motor_inverse", steer_motor_inverse);
    for (size_t i = 0; i < 4; i++) {
      steer_motor_dir_[i] = steer_motor_inverse[i] ? -1.0 : 1.0;
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
      } else if (name == "steer_gear_ratio") {
        steer_gear_ratio_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: steer_gear_ratio = %.3f", steer_gear_ratio_);
      } else if (name == "wheel_speed_limit") {
        wheel_speed_limit_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: wheel_speed_limit = %.3f", wheel_speed_limit_);
      } else if (name == "steer_angle_limit") {
        steer_angle_limit_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: steer_angle_limit = %.3f", steer_angle_limit_);
      } else if (name == "angle_diff_range") {
        angle_diff_range_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: angle_diff_range = %.3f", angle_diff_range_);
      } else if (name == "steer_theta_offset") {
        steer_theta_offset_ = parameter.as_double_array();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: steer_theta_offset = [%f, %f, %f, %f]",
          steer_theta_offset_[0], steer_theta_offset_[1], steer_theta_offset_[2],
          steer_theta_offset_[3]);
      } else if (name == "wheel_motor_inverse") {
        std::vector<bool> wheel_motor_inverse = parameter.as_bool_array();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: wheel_motor_inverse = [%s, %s, %s, %s]",
          wheel_motor_inverse[0] ? "true" : "false", wheel_motor_inverse[1] ? "true" : "false",
          wheel_motor_inverse[2] ? "true" : "false", wheel_motor_inverse[3] ? "true" : "false");
        for (int i = 0; i < N; i++) {
          wheel_motor_dir_[i] = wheel_motor_inverse[i] ? -1.0 : 1.0;
        }
      } else if (name == "steer_motor_inverse") {
        std::vector<bool> steer_motor_inverse = parameter.as_bool_array();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: steer_motor_inverse = [%s, %s, %s, %s]",
          steer_motor_inverse[0] ? "true" : "false", steer_motor_inverse[1] ? "true" : "false",
          steer_motor_inverse[2] ? "true" : "false", steer_motor_inverse[3] ? "true" : "false");
        for (int i = 0; i < N; i++) {
          steer_motor_dir_[i] = steer_motor_inverse[i] ? -1.0 : 1.0;
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

    const double heading = use_imu_ ? yaw_ : 0.0;
    calculate_swerve_drive(vx_ref, vy_ref, omega_ref, heading);
    swerve_drive_ref_publisher_->publish(swerve_drive_ref_);
    RCLCPP_INFO(
      this->get_logger(), "Received cmd_vel: vx=%.3f, vy=%.3f, omega=%.3f", vx_ref, vy_ref,
      omega_ref);
    RCLCPP_INFO(
      this->get_logger(),
      "Calculated swerve drive ref: omega0=%.3f, theta0=%.3f, omega1=%.3f, "
      "theta1=%.3f, omega2=%.3f, theta2=%.3f, omega3=%.3f, theta3=%.3f",
      swerve_drive_ref_.omega0, swerve_drive_ref_.theta0, swerve_drive_ref_.omega1,
      swerve_drive_ref_.theta1, swerve_drive_ref_.omega2, swerve_drive_ref_.theta2,
      swerve_drive_ref_.omega3, swerve_drive_ref_.theta3);
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
    yaw_raw_ = yaw;
    yaw_ = angle_normalize(yaw + yaw_offset_);
    // RCLCPP_INFO(this->get_logger(), "yaw = %f", yaw_);
  }

  void set_swerve_drive_yaw_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // imuを使わない設定の場合は処理しない
    if (!use_imu_) return;

    yaw_offset_ = msg->data - yaw_raw_;
    RCLCPP_INFO(this->get_logger(), "Set swerve drive yaw = %f", msg->data);
  }

  void manual_swerve_drive_ref_callback(const r1_msgs::msg::SwerveDrive::SharedPtr msg)
  {
    // 回転方向とギア比を考慮し代入。limitの確認は手動なので、行わない
    swerve_drive_ref_.omega0 = wheel_motor_dir_[0] * msg->omega0 / wheel_gear_ratio_;
    swerve_drive_ref_.omega1 = wheel_motor_dir_[1] * msg->omega1 / wheel_gear_ratio_;
    swerve_drive_ref_.omega2 = wheel_motor_dir_[2] * msg->omega2 / wheel_gear_ratio_;
    swerve_drive_ref_.omega3 = wheel_motor_dir_[3] * msg->omega3 / wheel_gear_ratio_;
    swerve_drive_ref_.theta0 =
      (steer_motor_dir_[0] * msg->theta0 + steer_theta_offset_[0]) / steer_gear_ratio_;
    swerve_drive_ref_.theta1 =
      (steer_motor_dir_[1] * msg->theta1 + steer_theta_offset_[1]) / steer_gear_ratio_;
    swerve_drive_ref_.theta2 =
      (steer_motor_dir_[2] * msg->theta2 + steer_theta_offset_[2]) / steer_gear_ratio_;
    swerve_drive_ref_.theta3 =
      (steer_motor_dir_[3] * msg->theta3 + steer_theta_offset_[3]) / steer_gear_ratio_;
    // publish
    swerve_drive_ref_publisher_->publish(swerve_drive_ref_);
    // ステアの前回値を更新
    prev_steer_theta_[0] = msg->theta0;
    prev_steer_theta_[1] = msg->theta1;
    prev_steer_theta_[2] = msg->theta2;
    prev_steer_theta_[3] = msg->theta3;

    RCLCPP_INFO(
      this->get_logger(),
      "Manual swerve drive ref: omega0=%.3f, theta0=%.3f, omega1=%.3f, "
      "theta1=%.3f, omega2=%.3f, theta2=%.3f, omega3=%.3f, theta3=%.3f",
      swerve_drive_ref_.omega0, swerve_drive_ref_.theta0, swerve_drive_ref_.omega1,
      swerve_drive_ref_.theta1, swerve_drive_ref_.omega2, swerve_drive_ref_.theta2,
      swerve_drive_ref_.omega3, swerve_drive_ref_.theta3);
  }

  void calculate_swerve_drive(double vx_ref, double vy_ref, double omega_ref, double theta)
  {
    double wheel_vx[4];
    double wheel_vy[4];
    double wheel_v[4];
    double steer_theta[4];
    double R = robot_radius_;

    double zero_vector_threshold = 1e-3;

    // 速度指令がほぼゼロのときは、前回のステアリング角度を維持するようにする
    if (
      std::abs(vx_ref) < zero_vector_threshold && std::abs(vy_ref) < zero_vector_threshold &&
      std::abs(omega_ref) < zero_vector_threshold) {
      // 旋回角は前回と同じ値、速度はゼロにする
      RCLCPP_INFO(
        this->get_logger(), "Velocity command is near zero, maintaining previous steering angles.");
      swerve_drive_ref_ = prev_swerve_drive_ref_;
      swerve_drive_ref_.omega0 = 0.0;
      swerve_drive_ref_.omega1 = 0.0;
      swerve_drive_ref_.omega2 = 0.0;
      swerve_drive_ref_.omega3 = 0.0;
      return;
    }

    // 4輪独立ステアリングの逆運動学の計算
    for (int i = 0; i < 4; i++) {
      // 逆運動学を計算
      wheel_vx[i] = vx_ref - R * omega_ref * std::sin(theta + i * M_PI / 2.0);
      wheel_vy[i] = vy_ref + R * omega_ref * std::cos(theta + i * M_PI / 2.0);
      wheel_v[i] = std::sqrt(std::pow(wheel_vx[i], 2) + std::pow(wheel_vy[i], 2));
      // thetaが連続となるようにする。
      // 前回の指令値との角度差を計算する
      steer_theta[i] = std::atan2(wheel_vy[i], wheel_vx[i]);
      double diff = angle_diff(steer_theta[i], prev_steer_theta_[i]);
      if (std::abs(diff) < angle_diff_range_) {
        // 角度差が一定値以下のときは、連続となるようにする。
        while (steer_theta[i] - prev_steer_theta_[i] > M_PI) {
          steer_theta[i] -= 2 * M_PI;
        }
        while (steer_theta[i] - prev_steer_theta_[i] < -M_PI) {
          steer_theta[i] += 2 * M_PI;
        }
      }
    }

    // 計算した角速度がlimitより高いかを確認
    double max_omega = std::abs(wheel_v[0] / wheel_radius_);
    for (int i = 1; i < 4; i++) {
      max_omega = std::max(max_omega, std::abs(wheel_v[i] / wheel_radius_));
    }
    if (max_omega > wheel_speed_limit_) {
      const double scale = wheel_speed_limit_ / max_omega;
      for (int i = 0; i < 4; i++) {
        wheel_v[i] *= scale;
      }
      RCLCPP_ERROR(
        this->get_logger(), "Wheel omega limit exceeded. Scaling down by factor %.3f", scale);
    }

    // 計算したステアリング角度がlimitより高いかを確認
    // ステアリングの角度制限を超えたらクランプしてしまうと、それはそれで危ないので、ERRORを出すだけにする。
    for (int i = 0; i < 4; i++) {
      if (std::abs(steer_theta[i]) > steer_angle_limit_) {
        RCLCPP_ERROR(
          this->get_logger(), "steer angle limit exceeded for wheel %d: %.3f rad", i,
          steer_theta[i]);
      }
    }

    // 回転方向とギア比を考慮し代入
    swerve_drive_ref_.omega0 = wheel_motor_dir_[0] * wheel_v[0] / wheel_radius_ / wheel_gear_ratio_;
    swerve_drive_ref_.omega1 = wheel_motor_dir_[1] * wheel_v[1] / wheel_radius_ / wheel_gear_ratio_;
    swerve_drive_ref_.omega2 = wheel_motor_dir_[2] * wheel_v[2] / wheel_radius_ / wheel_gear_ratio_;
    swerve_drive_ref_.omega3 = wheel_motor_dir_[3] * wheel_v[3] / wheel_radius_ / wheel_gear_ratio_;
    swerve_drive_ref_.theta0 =
      (steer_motor_dir_[0] * steer_theta[0] + steer_theta_offset_[0]) / steer_gear_ratio_;
    swerve_drive_ref_.theta1 =
      (steer_motor_dir_[1] * steer_theta[1] + steer_theta_offset_[1]) / steer_gear_ratio_;
    swerve_drive_ref_.theta2 =
      (steer_motor_dir_[2] * steer_theta[2] + steer_theta_offset_[2]) / steer_gear_ratio_;
    swerve_drive_ref_.theta3 =
      (steer_motor_dir_[3] * steer_theta[3] + steer_theta_offset_[3]) / steer_gear_ratio_;

    // 前回値を更新
    for (int i = 0; i < 4; i++) {
      prev_steer_theta_[i] = steer_theta[i];
    }
    prev_swerve_drive_ref_ = swerve_drive_ref_;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<r1_msgs::msg::SwerveDrive>::SharedPtr swerve_drive_ref_publisher_;
  rclcpp::Subscription<r1_msgs::msg::SwerveDrive>::SharedPtr manual_swerve_drive_ref_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_swerve_drive_yaw_subscription_;
  r1_msgs::msg::SwerveDrive swerve_drive_ref_;
  r1_msgs::msg::SwerveDrive prev_swerve_drive_ref_;
  double prev_steer_theta_[4] = {0};

  // ========== 機械パラメータ ==========
  static constexpr int N = 4;
  double robot_length_;      // ロボットの長さ (m)
  double robot_width_;       // ロボットの幅 (m)
  double robot_radius_;      // ロボットの半径 (m) = sqrt((length/2)^2 + (width/2)^2)
  double wheel_radius_;      // ホイールの半径 (m)
  double wheel_gear_ratio_;  // ホイールのギア比（減速比）。ギア比で割った値が出力される
  double steer_gear_ratio_;  // ステアリングのギア比（減速比）。ギア比で割った値が出力される
  // ========== 制御パラメータ ==========
  double wheel_speed_limit_;  // ホイールの速度制限 (rad/s)
  double steer_angle_limit_;  // ステアリング角度の制限 (rad)
  double
    angle_diff_range_;  // 角度差の範囲。この値を小さくすると、ステアリングの角度が連続になるようにする範囲が狭くなる。単位はrad。
  std::vector<double> steer_theta_offset_ = {
    0.0, 0.0, 0.0, 0.0};  // ステアリングの角度オフセット (rad)
  // motor_inverse = trueのとき、motor_dir_が-1.0になる。
  std::vector<double> wheel_motor_dir_ = {1.0, 1.0, 1.0, 1.0};
  std::vector<double> steer_motor_dir_ = {1.0, 1.0, 1.0, 1.0};
  bool use_imu_ = true;      // IMUを使用するかどうか
  double yaw_offset_ = 0.0;  // IMUのyaw角のオフセット (rad)
  double yaw_ = 0.0;         // IMUのyaw角
  double yaw_raw_ = 0.0;     // IMUのyaw角（オフセットなし）
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
