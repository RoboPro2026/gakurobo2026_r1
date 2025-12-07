/**
 * @file bno086_subscription_node.cpp
 * @author Yamaguchi Yudai
 * @brief bno086_nodeからsensor_msgs/Imuメッセージを購読するサンプルノード
 * @version 0.1
 * @date 2025-10-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("bno086_subscription_node")
  {
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "bno086/imu/data_raw", 10, std::bind(&MyNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double yaw, pitch, roll;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "Yaw: %.3f, Pitch: %.3f, Roll: %.3f", yaw, pitch, roll);
    RCLCPP_INFO(
      this->get_logger(), "Angular Velocity - x: %.3f, y: %.3f, z: %.3f", msg->angular_velocity.x,
      msg->angular_velocity.y, msg->angular_velocity.z);
    RCLCPP_INFO(
      this->get_logger(), "Linear Acceleration - x: %.3f, y: %.3f, z: %.3f",
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}