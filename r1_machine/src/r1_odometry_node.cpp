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

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_odometry_node")
  {
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

    encoder_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/odometry_encoder", 10, std::bind(&MyNode::encoder_callback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/bno086/imu/data_raw", 10, std::bind(&MyNode::imu_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));

    prev_time_ = this->now();

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("offset_pos_x", 0.0);
    this->declare_parameter<double>("offset_pos_y", 0.0);
    this->declare_parameter<double>("offset_yaw", 0.0);

    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("offset_pos_x", offset_pos_x_);
    this->get_parameter("offset_pos_y", offset_pos_y_);
    this->get_parameter("offset_yaw", offset_yaw_);
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
      } else {
        result.successful = false;
        result.reason = "Invalid parameter name: " + name;
        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
      }
    }
    return result;
  }

  void encoder_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // エンコーダの値を更新
    encoder_x_ = msg->data[0];
    encoder_y_ = msg->data[1];
    prev_time_ = this->now();
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // IMUのyaw角の情報を更新、他の情報は使用しない（必要ないので）
    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double yaw, pitch, roll;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    imu_yaw_ = yaw;
    imu_yaw_angular_velocity_ = msg->angular_velocity.z;
  }

  void timer_callback()
  {
    // 時間差を計算
    rclcpp::Time current_time = this->now();
    if (current_time.get_clock_type() != prev_time_.get_clock_type()) {
      prev_time_ = current_time;
      return;
    }
    double dt_sec = (current_time - prev_time_).seconds();

    if (dt_sec <= 0.0) {
      prev_time_ = current_time;
      return;
    }

    // オドメトリメッセージを作成
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // 位置と姿勢の更新
    pos_x_ = wheel_radius_ * encoder_x_;
    pos_y_ = wheel_radius_ * encoder_y_;

    odom_msg.pose.pose.position.x = pos_x_ + offset_pos_x_;
    odom_msg.pose.pose.position.y = pos_y_ + offset_pos_y_;
    odom_msg.pose.pose.position.z = imu_yaw_ + offset_yaw_;

    odom_msg.twist.twist.linear.x = (pos_x_ - prev_pos_x_) / dt_sec;
    odom_msg.twist.twist.linear.y = (pos_y_ - prev_pos_y_) / dt_sec;
    odom_msg.twist.twist.angular.z = imu_yaw_angular_velocity_;

    RCLCPP_INFO(
      this->get_logger(),
      "position(x = %.3f, y = %.3f, yaw = %.3f)\nvelocity(vx = %.3f, vy = %.3f, omega = %.3f)",
      odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z,
      odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z);

    // オドメトリを配信
    odometry_publisher_->publish(odom_msg);

    // 前回の値を更新
    prev_time_ = current_time;
    prev_pos_x_ = pos_x_;
    prev_pos_y_ = pos_y_;
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr encoder_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time prev_time_ = rclcpp::Time(0);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  // マイコンから送られてくるエンコーダの値は、すでに積分されたものが送られてくる。単位は[rad]
  double encoder_x_ = 0.0;                 // rad
  double encoder_y_ = 0.0;                 // rad
  double pos_x_ = 0.0;                     // m
  double pos_y_ = 0.0;                     // m
  double prev_pos_x_ = 0.0;                // m
  double prev_pos_y_ = 0.0;                // m
  double imu_yaw_ = 0.0;                   // rad
  double imu_yaw_angular_velocity_ = 0.0;  // rad/s
  double wheel_radius_ = 0.05;             // m
  double offset_pos_x_ = 0.0;              // m
  double offset_pos_y_ = 0.0;              // m
  double offset_yaw_ = 0.0;                // rad
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
