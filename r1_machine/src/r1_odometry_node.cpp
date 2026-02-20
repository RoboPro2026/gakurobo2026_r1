/**
 * @file r1_odometry_node.cpp
 * @author Yamaguchi Yudai
 * @brief 設置エンコーダの値からオドメトリを計算してnav_msgs/Odometryで配信するノード
 * @version 0.1
 * @date 2025-10-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "r1_msgs/msg/odometry_encoder.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_odometry_node")
  {
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

    encoder_subscription_ = this->create_subscription<r1_msgs::msg::OdometryEncoder>(
      "/odometry_encoder", 10, std::bind(&MyNode::encoder_callback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/bno086/imu/data_raw", 10, std::bind(&MyNode::imu_callback, this, std::placeholders::_1));

    imu_offset_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/odometry_offset", 10,
      std::bind(&MyNode::odometry_offset_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    this->declare_parameter<double>("wheel_radius", 0.025);
    this->declare_parameter<double>("offset_pos_x", 0.0);
    this->declare_parameter<double>("offset_pos_y", 0.0);
    this->declare_parameter<double>("offset_yaw", 0.0);
    this->declare_parameter<bool>("encoder_x_inverse", false);
    this->declare_parameter<bool>("encoder_y_inverse", false);
    this->declare_parameter<bool>("use_imu", true);

    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("offset_pos_x", offset_pos_x_);
    this->get_parameter("offset_pos_y", offset_pos_y_);
    this->get_parameter("offset_yaw", offset_yaw_);

    bool encoder_inverse[2];
    this->get_parameter("encoder_x_inverse", encoder_inverse[0]);
    this->get_parameter("encoder_y_inverse", encoder_inverse[1]);
    encoder_x_direction_ = encoder_inverse[0] ? -1.0 : 1.0;
    encoder_y_direction_ = encoder_inverse[1] ? -1.0 : 1.0;

    this->get_parameter("use_imu", use_imu_);
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      const auto & name = param.get_name();
      if (name == "wheel_radius") {
        wheel_radius_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: wheel_radius = %.3f", wheel_radius_);
      } else if (name == "offset_pos_x") {
        offset_pos_x_ += param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: offset_pos_x = %.3f", offset_pos_x_);
      } else if (name == "offset_pos_y") {
        offset_pos_y_ += param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: offset_pos_y = %.3f", offset_pos_y_);
      } else if (name == "offset_yaw") {
        offset_yaw_ += param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: offset_yaw = %.3f", offset_yaw_);
      } else if (name == "encoder_x_inverse") {
        encoder_x_direction_ = param.as_bool() ? -1.0 : 1.0;
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: encoder_x_inverse = %s",
          param.as_bool() ? "true" : "false");
      } else if (name == "encoder_y_inverse") {
        encoder_y_direction_ = param.as_bool() ? -1.0 : 1.0;
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: encoder_y_inverse = %s",
          param.as_bool() ? "true" : "false");
      } else if (name == "use_imu") {
        use_imu_ = param.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: use_imu = %s",
          param.as_bool() ? "true" : "false");
      } else {
        result.successful = false;
        result.reason = "Invalid parameter name: " + name;
        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
      }
    }
    return result;
  }

  void encoder_callback(const r1_msgs::msg::OdometryEncoder::SharedPtr msg)
  {
    // エンコーダの値を更新
    encoder_update_ = true;
    encoder_pos_x_ = msg->encoder_pos_x;
    encoder_pos_y_ = msg->encoder_pos_y;
    encoder_speed_x_ = msg->encoder_speed_x;
    encoder_speed_y_ = msg->encoder_speed_y;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // IMUのyaw角の情報を更新、他の情報は使用しない（必要ないので）

    // imuを使わない設定の場合は処理しない
    if (!use_imu_) return;

    imu_update_ = true;
    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double yaw, pitch, roll;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    imu_yaw_ = yaw;
    imu_yaw_angular_velocity_ = msg->angular_velocity.z;
  }

  void odometry_offset_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // odometryのオフセット値を更新
    if (msg->data.size() < 3) {
      RCLCPP_ERROR(this->get_logger(), "Odometry offset message must contain at least 3 elements");
      return;
    }
    offset_pos_x_ += msg->data[0];
    offset_pos_y_ += msg->data[1];
    offset_yaw_ += msg->data[2];
    RCLCPP_INFO(
      this->get_logger(),
      "Updated Odometry offsets: offset_pos_x = %.3f, offset_pos_y = %.3f, offset_yaw = %.3f",
      offset_pos_x_, offset_pos_y_, offset_yaw_);
  }

  void timer_callback()
  {
    // エンコーダとIMUの両方のデータが更新されていなければ処理しない
    if (encoder_update_ == false) {
      return;
    }

    if (use_imu_ && imu_update_ == false) {
      return;
    }

    // オドメトリメッセージを作成
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // 位置と姿勢の更新
    double vx = encoder_x_direction_ * wheel_radius_ * encoder_speed_x_;
    double vy = encoder_y_direction_ * wheel_radius_ * encoder_speed_y_;
    double yaw = imu_yaw_ + offset_yaw_;
    double vx_world = vx * std::cos(yaw) - vy * std::sin(yaw);
    double vy_world = vx * std::sin(yaw) + vy * std::cos(yaw);
    double dt = 0.01;
    double pos_x = vx_world * dt;
    double pos_y = vy_world * dt;

    odom_msg.pose.pose.position.x = pos_x + offset_pos_x_;
    odom_msg.pose.pose.position.y = pos_y + offset_pos_y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);  // roll, pitch, yaw
    q.normalize();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = encoder_x_direction_ * wheel_radius_ * encoder_speed_x_;
    odom_msg.twist.twist.linear.y = encoder_y_direction_ * wheel_radius_ * encoder_speed_y_;
    odom_msg.twist.twist.angular.z = imu_yaw_angular_velocity_;

    RCLCPP_INFO(
      this->get_logger(),
      "position(x = %.3f, y = %.3f, yaw = %.3f) velocity(vx = %.3f, vy = %.3f, omega "
      "= %.3f)",
      odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, imu_yaw_ + offset_yaw_,
      odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z);

    // オドメトリを配信
    odometry_publisher_->publish(odom_msg);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<r1_msgs::msg::OdometryEncoder>::SharedPtr encoder_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr imu_offset_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  // マイコンから送られてくるエンコーダの値は、すでに積分されたものが送られてくる。単位は[rad]
  double encoder_pos_x_ = 0.0;             // rad
  double encoder_pos_y_ = 0.0;             // rad
  double encoder_speed_x_ = 0.0;           // rad/s
  double encoder_speed_y_ = 0.0;           // rad/s
  double imu_yaw_ = 0.0;                   // rad
  double imu_yaw_angular_velocity_ = 0.0;  // rad/s
  double wheel_radius_ = 0.025;            // m
  double offset_pos_x_ = 0.0;              // m
  double offset_pos_y_ = 0.0;              // m
  double offset_yaw_ = 0.0;                // rad
  // encoder_inverse = trueのとき、motor_dir_が-1.0になる。
  double encoder_x_direction_ = 1.0;
  double encoder_y_direction_ = 1.0;
  bool encoder_update_ = true;
  bool imu_update_ = true;
  bool use_imu_ = true;
  // TODO: 必要であれば、imu_yaw_angular_velocity_のオフセットも追加する
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
