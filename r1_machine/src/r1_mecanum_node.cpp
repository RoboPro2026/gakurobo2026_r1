/**
 * @file r1_mecanum_node.cpp
 * @author Yamaguchi Yudai
 * @brief メカナムホイールの運動学/逆運動学計算するノード
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
  //  FL(0) ↖︎       |          ↙ FR(1)
  //        \------------------/
  //        |        |         |
  //        |        |         |
  //        |                  | 
  //        |        |         |
  // <---x--|--------O---------|
  //        |       ↺ w       |
  //        |                  | 
  //        |                  |
  //        |                  | 
  //        /------------------\
  //  RL(2) ↙                  ↖︎ RR(3)
// clang-format on

#include <chrono>
#include <limits>

#include "geometry_msgs/msg/twist.hpp"
#include "r1_msgs/msg/mecanum.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_mecanum_node")
  {
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MyNode::cmd_vel_callback, this, std::placeholders::_1));

    // 角速度指令値
    wheel_speeds_ref_publisher_ =
      this->create_publisher<r1_msgs::msg::Mecanum>("/mecanum_wheel_speeds_ref", 10);

    wheel_speeds_feedback_subscription_ = this->create_subscription<r1_msgs::msg::Mecanum>(
      "/mecanum_wheel_speeds_feedback", 10,
      std::bind(&MyNode::wheel_speeds_feedback_callback, this, std::placeholders::_1));

    feedback_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/mecanum_feedback_vel", 10);

    parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    // BNO086以外のIMUを使う場合は、適宜変えること。
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/bno086/imu/data_raw", 10, std::bind(&MyNode::imu_callback, this, std::placeholders::_1));

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
    this->declare_parameter("gear_ratio", 1.0, parameter_descriptor);
    this->declare_parameter("motor_inverse", std::vector<bool>{false, false, false, false});
    this->declare_parameter("use_imu", true);

    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("robot_length", robot_length_);
    this->get_parameter("robot_width", robot_width_);
    this->get_parameter("speed_limit", speed_limit_);
    this->get_parameter("gear_ratio", gear_ratio_);
    std::vector<bool> motor_inverse(4);
    this->get_parameter("motor_inverse", motor_inverse);
    for (int i = 0; i < 4; i++) motor_dir_[i] = motor_inverse[i] ? -1.0 : 1.0;
    this->get_parameter("use_imu", use_imu_);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    target_vel_ = *msg;
    // メカナムホイールの逆運動学を計算
    calculate_wheel_speeds(
      target_vel_.linear.x, target_vel_.linear.y, target_vel_.angular.z, theta_);

    // 角速度をFloat64MultiArrayでパブリッシュ
    auto mecanum_msg = r1_msgs::msg::Mecanum();
    mecanum_msg.fl_wheel_speed = wheel_speeds_ref_[FL];
    mecanum_msg.fr_wheel_speed = wheel_speeds_ref_[FR];
    mecanum_msg.rl_wheel_speed = wheel_speeds_ref_[RL];
    mecanum_msg.rr_wheel_speed = wheel_speeds_ref_[RR];
    wheel_speeds_ref_publisher_->publish(mecanum_msg);

    // デバッグ用
    RCLCPP_INFO(
      this->get_logger(), "Cmd Vel: x: %.2f, y: %.2f, omega: %.2f", target_vel_.linear.x,
      target_vel_.linear.y, target_vel_.angular.z);
    RCLCPP_INFO(
      this->get_logger(), "Wheel Speeds Ref: FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
      wheel_speeds_ref_[FL], wheel_speeds_ref_[FR], wheel_speeds_ref_[RL], wheel_speeds_ref_[RR]);
  }

  void wheel_speeds_feedback_callback(const r1_msgs::msg::Mecanum::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(), "Wheel Speeds Feedback: FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
      msg->fl_wheel_speed, msg->fr_wheel_speed, msg->rl_wheel_speed, msg->rr_wheel_speed);
    auto wheel_speeds_feedback = std::vector<double>{
      msg->fl_wheel_speed, msg->fr_wheel_speed, msg->rl_wheel_speed, msg->rr_wheel_speed};
    // メカナムホイールの順運動学を計算
    auto res = calculate_robot_velocity(wheel_speeds_feedback, theta_);
    auto feedback_vel = geometry_msgs::msg::Twist();
    feedback_vel.linear.x = res[0];
    feedback_vel.linear.y = res[1];
    feedback_vel.angular.z = res[2];
    feedback_vel_publisher_->publish(feedback_vel);
    RCLCPP_INFO(
      this->get_logger(), "Feedback Vel: x: %.2f, y: %.2f, omega: %.2f", feedback_vel.linear.x,
      feedback_vel.linear.y, feedback_vel.angular.z);
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
      } else if (name == "gear_ratio") {
        gear_ratio_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: gear_ratio = %.3f", gear_ratio_);
      } else if (name == "use_imu") {
        use_imu_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: use_imu = %s",
          parameter.as_bool() ? "true" : "false");
      } else if (name == "motor_inverse") {
        std::vector<bool> motor_inverse = parameter.as_bool_array();
        for (int i = 0; i < 4; i++) motor_dir_[i] = motor_inverse[i] ? -1.0 : 1.0;
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: motor_inverse = [%s, %s, %s, %s]",
          motor_inverse[0] ? "true" : "false", motor_inverse[1] ? "true" : "false",
          motor_inverse[2] ? "true" : "false", motor_inverse[3] ? "true" : "false");
      } else {
        result.successful = false;
        result.reason = "Invalid parameter name: " + name;
        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
      }
    }

    return result;
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
    theta_ = yaw;
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
    // ここはうまく行かなかったら適当に入れ替えてる
    wheel_speeds_ref_[FL] = (1 / R) * (vx - vy + (L + W) * omega);
    wheel_speeds_ref_[FR] = (1 / R) * (vx + vy + (L + W) * omega);
    wheel_speeds_ref_[RL] = (1 / R) * (vx + vy - (L + W) * omega);
    wheel_speeds_ref_[RR] = (1 / R) * (vx - vy - (L + W) * omega);

    // wheel_speeds_ref_[FL] = (1 / R) * (vx + vy + (L + W) * omega);
    // wheel_speeds_ref_[FR] = (1 / R) * (vx - vy + (L + W) * omega);
    // wheel_speeds_ref_[RL] = (1 / R) * (vx - vy - (L + W) * omega);
    // wheel_speeds_ref_[RR] = (1 / R) * (vx + vy - (L + W) * omega);

    // モーターのギア比と回転方向を考慮
    for (int i = 0; i < 4; i++) {
      // ギア比
      wheel_speeds_ref_[i] /= gear_ratio_;
      // 回転方向
      wheel_speeds_ref_[i] *= motor_dir_[i];
    }

    // 計算した値がlimitより高いかを確認
    max_speed = std::abs(wheel_speeds_ref_[FL]);
    for (int i = 0; i < 4; i++) {
      max_speed = std::max(max_speed, std::abs(wheel_speeds_ref_[i]));
    }

    // 計算した値がlimitを超えていたら、limitに収まるように全体をスケーリング
    if (max_speed > speed_limit_) {
      for (int i = 0; i < 4; i++) {
        wheel_speeds_ref_[i] = wheel_speeds_ref_[i] / max_speed * speed_limit_;
      }
    }
  }

  /**
 * @brief メカナムホイールの順運動学を計算する
 *
 * @param wheel_speed FL, FR, RL, RR の順のホイール角速度[rad/s]
 * @param theta ロボットの角度[rad]。y軸正方向を0度とし、反時計回りを正とする。IMUを使用しない場合はtheta=0
 */
  std::vector<double> calculate_robot_velocity(std::vector<double> _wheel_speed, double theta)
  {
    double L = robot_length_;
    double W = robot_width_;
    double R = wheel_radius_;
    double vx, vy, omega;

    std::vector<double> wheel_speed(N);
    // モータの回転方向とギア比を考慮
    for (int i = 0; i < 4; i++) {
      wheel_speed[i] = motor_dir_[i] * _wheel_speed[i] * gear_ratio_;
    }

    // 順運動学計算
    vx = (R / 4.0) * (wheel_speed[FL] + wheel_speed[FR] + wheel_speed[RL] + wheel_speed[RR]);
    vy = (R / 4.0) * (wheel_speed[FL] - wheel_speed[FR] - wheel_speed[RL] + wheel_speed[RR]);
    omega = (R / (4.0 * (L + W))) *
            (wheel_speed[FL] + wheel_speed[FR] - wheel_speed[RL] - wheel_speed[RR]);

    // IMU角度による座標変換（θはロボット姿勢角）
    // TODO: ここの回転行列の符号が逆な気がする
    double vx_robot = vx;
    double vy_robot = vy;
    vx = vx_robot * cos(theta) - vy_robot * sin(theta);
    vy = vx_robot * sin(theta) + vy_robot * cos(theta);

    std::vector<double> ret = {vx, vy, omega};
    return ret;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;
  rclcpp::Publisher<r1_msgs::msg::Mecanum>::SharedPtr wheel_speeds_ref_publisher_;
  rclcpp::Subscription<r1_msgs::msg::Mecanum>::SharedPtr wheel_speeds_feedback_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr feedback_vel_publisher_;
  // 速度指令値
  geometry_msgs::msg::Twist target_vel_;
  double theta_ = 0.0;
  bool use_imu_ = true;
  double speed_limit_;   //rad/s
  double robot_length_;  // ロボットの長さ (m)
  double robot_width_;   // ロボットの幅 (m)
  double wheel_radius_;  // ホイールの半径 (m)
  double gear_ratio_;    // ギア比（減速比）。ギア比で割った値が出力される
  // motor_inverse = trueのとき、motor_dir_が-1.0になる。
  std::vector<double> motor_dir_ = {1.0, 1.0, 1.0, 1.0};
  std::vector<double> wheel_speeds_ref_ = {0.0, 0.0, 0.0, 0.0};  // FL, FR, RL, RR
  static constexpr int FL = 0;
  static constexpr int FR = 1;
  static constexpr int RL = 2;
  static constexpr int RR = 3;
  static constexpr int N = 4;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
