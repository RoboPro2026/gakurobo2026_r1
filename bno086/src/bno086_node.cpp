/**
 * @file bno086_node.cpp
 * @author Yamaguchi Yudai
 * @brief bno086のROS 2ノード
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <chrono>

#include "bno086/bno086_driver.h"
#include "bno086/serial_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("bno086_node")
  {
    // パラメータを宣言
    this->declare_parameter<std::string>("port");
    // パラメータを取得
    std::string port_name = this->get_parameter("port").as_string();

    RCLCPP_INFO(this->get_logger(), "Connecting to port: %s", port_name.c_str());

    // 取得したパラメータで初期化
    try {
      serial_ = std::make_shared<SerialDriver>(port_name);
      if (serial_->get_is_initialize_success() == false) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open port: %s.", port_name.c_str());
        rclcpp::shutdown();
        return;  // コンストラクタが終了すれば main も終了する
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to open port: %s. Error: %s", port_name.c_str(), e.what());
      // rclcpp::shutdown() を呼ぶか、例外を投げて終了させる
      rclcpp::shutdown();
      return;  // コンストラクタが終了すれば main も終了する
    }

    bno086_driver_ = std::make_shared<BNO086Driver>(serial_);
    // 100Hz
    timer_publisher_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("bno086/imu/data_raw", 10);

    this->declare_parameter<double>("offset_roll_angle", 0.0);
    this->declare_parameter<double>("offset_pitch_angle", 0.0);
    this->declare_parameter<double>("offset_yaw_angle", 0.0);
    this->declare_parameter<double>("offset_roll_angular_velocity", 0.0);
    this->declare_parameter<double>("offset_pitch_angular_velocity", 0.0);
    this->declare_parameter<double>("offset_yaw_angular_velocity", 0.0);
    this->declare_parameter<double>("offset_x_axis_accel", 0.0);
    this->declare_parameter<double>("offset_y_axis_accel", 0.0);
    this->declare_parameter<double>("offset_z_axis_accel", 0.0);

    offset_data_.roll_angle = this->get_parameter("offset_roll_angle").as_double();
    offset_data_.pitch_angle = this->get_parameter("offset_pitch_angle").as_double();
    offset_data_.yaw_angle = this->get_parameter("offset_yaw_angle").as_double();
    offset_data_.roll_angular_velocity =
      this->get_parameter("offset_roll_angular_velocity").as_double();
    offset_data_.pitch_angular_velocity =
      this->get_parameter("offset_pitch_angular_velocity").as_double();
    offset_data_.yaw_angular_velocity =
      this->get_parameter("offset_yaw_angular_velocity").as_double();
    offset_data_.x_axis_accel = this->get_parameter("offset_x_axis_accel").as_double();
    offset_data_.y_axis_accel = this->get_parameter("offset_y_axis_accel").as_double();
    offset_data_.z_axis_accel = this->get_parameter("offset_z_axis_accel").as_double();

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::on_parameter_event, this, std::placeholders::_1));
  }

  void timer_callback(void)
  {
    bno086_driver_->update();
    auto data = bno086_driver_->get_data();
    bno086_driver_->print(data);
    // オイラー角からクオータニオンに変換
    tf2::Quaternion q;
    q.setRPY(data.roll_angle, data.pitch_angle, data.yaw_angle);
    // sensor_msgs::msg::Imu メッセージを作成してパブリッシュ
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "bno086_link";
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    imu_msg.angular_velocity.x = data.roll_angular_velocity;
    imu_msg.angular_velocity.y = data.pitch_angular_velocity;
    imu_msg.angular_velocity.z = data.yaw_angular_velocity;
    imu_msg.linear_acceleration.x = data.x_axis_accel;
    imu_msg.linear_acceleration.y = data.y_axis_accel;
    imu_msg.linear_acceleration.z = data.z_axis_accel;
    imu_publisher_->publish(imu_msg);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_event(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "offset_roll_angle") {
        offset_data_.roll_angle += parameter.as_double();
        bno086_driver_->set_offset_data(offset_data_);
        RCLCPP_INFO(this->get_logger(), "offset_roll_angle set to: %f", offset_data_.roll_angle);
      } else if (parameter.get_name() == "offset_pitch_angle") {
        offset_data_.pitch_angle += parameter.as_double();
        bno086_driver_->set_offset_data(offset_data_);
        RCLCPP_INFO(this->get_logger(), "offset_pitch_angle set to: %f", offset_data_.pitch_angle);
      } else if (parameter.get_name() == "offset_yaw_angle") {
        offset_data_.yaw_angle += parameter.as_double();
        bno086_driver_->set_offset_data(offset_data_);
        RCLCPP_INFO(this->get_logger(), "offset_yaw_angle set to: %f", offset_data_.yaw_angle);
      } else if (parameter.get_name() == "offset_roll_angular_velocity") {
        offset_data_.roll_angular_velocity += parameter.as_double();
        bno086_driver_->set_offset_data(offset_data_);
        RCLCPP_INFO(
          this->get_logger(), "offset_roll_angular_velocity set to: %f",
          offset_data_.roll_angular_velocity);
      } else if (parameter.get_name() == "offset_pitch_angular_velocity") {
        offset_data_.pitch_angular_velocity += parameter.as_double();
        bno086_driver_->set_offset_data(offset_data_);
        RCLCPP_INFO(
          this->get_logger(), "offset_pitch_angular_velocity set to: %f",
          offset_data_.pitch_angular_velocity);
      } else if (parameter.get_name() == "offset_yaw_angular_velocity") {
        offset_data_.yaw_angular_velocity += parameter.as_double();
        bno086_driver_->set_offset_data(offset_data_);
        RCLCPP_INFO(
          this->get_logger(), "offset_yaw_angular_velocity set to: %f",
          offset_data_.yaw_angular_velocity);
      } else if (parameter.get_name() == "offset_x_axis_accel") {
        offset_data_.x_axis_accel += parameter.as_double();
        bno086_driver_->set_offset_data(offset_data_);
        RCLCPP_INFO(
          this->get_logger(), "offset_x_axis_accel set to: %f", offset_data_.x_axis_accel);
      } else if (parameter.get_name() == "offset_y_axis_accel") {
        offset_data_.y_axis_accel += parameter.as_double();
        bno086_driver_->set_offset_data(offset_data_);
        RCLCPP_INFO(
          this->get_logger(), "offset_y_axis_accel set to: %f", offset_data_.y_axis_accel);
      } else if (parameter.get_name() == "offset_z_axis_accel") {
        offset_data_.z_axis_accel += parameter.as_double();
        bno086_driver_->set_offset_data(offset_data_);
        RCLCPP_INFO(
          this->get_logger(), "offset_z_axis_accel set to: %f", offset_data_.z_axis_accel);
      } else {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to set parameter. Unknown parameter: %s",
          parameter.get_name().c_str());
        result.successful = false;
        result.reason = "Unknown parameter: " + parameter.get_name();
        return result;
      }
    }
    return result;
  }

  std::shared_ptr<SerialDriver> serial_;
  std::shared_ptr<BNO086Driver> bno086_driver_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  BNO086Driver::Data offset_data_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}