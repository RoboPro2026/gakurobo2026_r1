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
#include <complex>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "r1_msgs/msg/odometry_encoder.hpp"
#include "r1_util/r1_util.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_odometry_node")
  {
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    encoder_subscription_ = this->create_subscription<r1_msgs::msg::OdometryEncoder>(
      "/odometry_encoder", 10, std::bind(&MyNode::encoder_callback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/bno086/imu/data_raw", 10, std::bind(&MyNode::imu_callback, this, std::placeholders::_1));

    set_odometry_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/set_odometry", 10, std::bind(&MyNode::set_odometry_callback, this, std::placeholders::_1));

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    this->declare_parameter<double>("timer_rate", 100.0);
    this->declare_parameter<double>("wheel_radius", 0.025);
    this->declare_parameter<bool>("encoder_x_inverse", false);
    this->declare_parameter<bool>("encoder_y_inverse", false);
    this->declare_parameter<bool>("use_imu", true);

    this->get_parameter("timer_rate", timer_rate_);
    this->get_parameter("wheel_radius", wheel_radius_);

    bool encoder_inverse[2];
    this->get_parameter("encoder_x_inverse", encoder_inverse[0]);
    this->get_parameter("encoder_y_inverse", encoder_inverse[1]);
    encoder_x_direction_ = encoder_inverse[0] ? -1.0 : 1.0;
    encoder_y_direction_ = encoder_inverse[1] ? -1.0 : 1.0;

    this->get_parameter("use_imu", use_imu_);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&MyNode::timer_callback, this));
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      const auto & name = param.get_name();
      if (name == "timer_rate") {
        if (param.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "timer_rate must be greater than 0.0";
          RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
          continue;
        }
        timer_rate_ = param.as_double();
        timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / timer_rate_),
          std::bind(&MyNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Updated parameter: timer_rate = %.3f", timer_rate_);
      } else if (name == "wheel_radius") {
        wheel_radius_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: wheel_radius = %.3f", wheel_radius_);
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
    prev_encoder_pos_x_ = encoder_pos_x_;
    prev_encoder_pos_y_ = encoder_pos_y_;
    encoder_pos_x_ = msg->encoder_pos_x;
    encoder_pos_y_ = msg->encoder_pos_y;
    encoder_speed_x_ = msg->encoder_speed_x;
    encoder_speed_y_ = msg->encoder_speed_y;
  }

  /**
   * @brief 角度を-pi~piの範囲に正規化する
   * 
   * @param angle 
   * @return double 
   */
  double angle_normalize(double angle)
  {
    std::complex<double> ret = std::polar(1.0, angle);
    return std::arg(ret);
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

  void set_odometry_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // odometryの値を設定
    if (msg->data.size() < 3) {
      RCLCPP_ERROR(this->get_logger(), "Set odometry message must contain at least 3 elements");
      return;
    }
    // xとyは直接値を書き換える
    pos_x_ = msg->data[0];
    pos_y_ = msg->data[1];
    // yawはオフセットを更新する
    offset_yaw_ = msg->data[2] - imu_yaw_;

    RCLCPP_INFO(
      this->get_logger(), "Set odometry: pos_x = %.3f, pos_y = %.3f, yaw = %.3f", msg->data[0],
      msg->data[1], msg->data[2]);
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
    // オフセットを加算するので、値が-pi~piの範囲に収まるように正規化
    double yaw = angle_normalize(imu_yaw_ + offset_yaw_);
    double vx_world = vx * std::cos(yaw) - vy * std::sin(yaw);
    double vy_world = vx * std::sin(yaw) + vy * std::cos(yaw);
    double px = encoder_x_direction_ * wheel_radius_ * (encoder_pos_x_ - prev_encoder_pos_x_);
    double py = encoder_y_direction_ * wheel_radius_ * (encoder_pos_y_ - prev_encoder_pos_y_);
    double px_world = px * std::cos(yaw) - py * std::sin(yaw);
    double py_world = px * std::sin(yaw) + py * std::cos(yaw);
    pos_x_ += px_world;
    pos_y_ += py_world;
    // pos_x_ += vx_world * 0.01;
    // pos_y_ += vy_world * 0.01;

    odom_msg.pose.pose.position.x = pos_x_;
    odom_msg.pose.pose.position.y = pos_y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);  // roll, pitch, yaw
    q.normalize();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx_world;
    odom_msg.twist.twist.linear.y = vy_world;
    odom_msg.twist.twist.angular.z = imu_yaw_angular_velocity_;

    RCLCPP_INFO(
      this->get_logger(),
      "position(x = %.3f, y = %.3f, yaw = %.3f) velocity(vx = %.3f, vy = %.3f, omega "
      "= %.3f)",
      odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw,
      odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z);

    // オドメトリを配信
    odometry_publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = odom_msg.header;
    odom_tf.child_frame_id = odom_msg.child_frame_id;
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<r1_msgs::msg::OdometryEncoder>::SharedPtr encoder_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  // マイコンから送られてくるエンコーダの値は、すでに積分されたものが送られてくる。単位は[rad]
  double encoder_pos_x_ = 0.0;       // rad
  double encoder_pos_y_ = 0.0;       // rad
  double prev_encoder_pos_x_ = 0.0;  // rad
  double prev_encoder_pos_y_ = 0.0;  // rad
  double encoder_speed_x_ = 0.0;     // rad/s
  double encoder_speed_y_ = 0.0;     // rad/s
  double pos_x_ = 0.0;
  double pos_y_ = 0.0;
  double imu_yaw_ = 0.0;                   // rad
  double imu_yaw_angular_velocity_ = 0.0;  // rad/s
  double timer_rate_ = 100.0;
  double wheel_radius_ = 0.025;  // m
  double offset_pos_x_ = 0.0;    // m
  double offset_pos_y_ = 0.0;    // m
  double offset_yaw_ = 0.0;      // rad
  // encoder_inverse = trueのとき、motor_dir_が-1.0になる。
  double encoder_x_direction_ = 1.0;
  double encoder_y_direction_ = 1.0;
  bool encoder_update_ = true;
  bool imu_update_ = true;
  bool use_imu_ = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
