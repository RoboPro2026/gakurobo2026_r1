/**
 * @file test_node.cpp
 * @brief PS4のjoy入力をそのままSabacanRobomasRefに流す簡易ノード
 */

#include <algorithm>
#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "r1_msgs/msg/motor.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_ref.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyToSabacanNode : public rclcpp::Node
{
public:
  JoyToSabacanNode() : Node("joy_to_sabacan_node")
  {
    // parameters
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/sabacan_robomas_ref1");
    scale_ = this->declare_parameter<double>("ref_scale", 5);  // [-1,1] stick -> [-scale,scale]
    board_id_ = this->declare_parameter<int>("board_id", 1);
    motor_status_prefix_ =
      this->declare_parameter<std::string>("motor_status_prefix", "/test_motor_status");

    sabacan_ref_pub_ =
      this->create_publisher<sabacan_msgs::msg::SabacanRobomasRef>(topic_name_, 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SensorDataQoS(),
      std::bind(&JoyToSabacanNode::joyCallback, this, std::placeholders::_1));

    const std::string status_topic = "/sabacan_robomas_status" + std::to_string(board_id_);
    status_sub_ = this->create_subscription<sabacan_msgs::msg::SabacanRobomasStatus>(
      status_topic, 10, std::bind(&JoyToSabacanNode::statusCallback, this, std::placeholders::_1));

    for (size_t i = 0; i < motor_pubs_.size(); ++i) {
      motor_pubs_[i] =
        this->create_publisher<r1_msgs::msg::Motor>(motor_status_prefix_ + std::to_string(i), 10);
    }

    RCLCPP_INFO(this->get_logger(), "publish to %s with scale %.2f", topic_name_.c_str(), scale_);
    RCLCPP_INFO(
      this->get_logger(), "subscribing %s and publishing r1_msgs::msg::Motor to %s{0-3}",
      status_topic.c_str(), motor_status_prefix_.c_str());
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // stick: motor0(LY), motor1(LX), motor2(RY), motor3(RX)
    constexpr std::array<int, 4> axis_index = {1, 0, 4, 3};
    const size_t max_index =
      static_cast<size_t>(*std::max_element(axis_index.begin(), axis_index.end()));
    if (msg->axes.size() <= max_index) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "joy axes size is too small: %zu",
        msg->axes.size());
      return;
    }

    for (size_t motor = 0; motor < axis_index.size(); ++motor) {
      publishMotor(static_cast<uint8_t>(motor), msg->axes[axis_index[motor]]);
    }
  }

  void statusCallback(const sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg)
  {
    const uint8_t motor_number = msg->motor_number;
    if (motor_number >= motor_pubs_.size()) return;

    r1_msgs::msg::Motor motor_msg;
    motor_msg.motor_type = msg->motor_type;
    motor_msg.control_type = msg->control_type;
    motor_msg.motor_state = msg->motor_state;
    motor_msg.torque = msg->torque;
    motor_msg.speed = msg->speed;
    motor_msg.pos = msg->pos;
    motor_msg.abs_pos = msg->abs_pos;
    motor_msg.abs_speed = msg->abs_speed;
    motor_msg.abs_turn_cnt = msg->abs_turn_cnt;
    motor_msg.vesc_voltage = msg->vesc_voltage;
    motor_msg.vesc_current = msg->vesc_current;
    motor_msg.vesc_speed = msg->vesc_speed;

    motor_pubs_[motor_number]->publish(motor_msg);
  }

  void publishMotor(uint8_t motor_number, float raw_axis)
  {
    sabacan_msgs::msg::SabacanRobomasRef ref_msg;
    ref_msg.motor_number = motor_number;
    ref_msg.ref = static_cast<float>(scale_ * raw_axis);
    sabacan_ref_pub_->publish(ref_msg);
    RCLCPP_INFO(
      this->get_logger(), "motor %d: axis %.3f -> ref %.3f", motor_number, raw_axis, ref_msg.ref);
  }

  std::string topic_name_;
  double scale_;
  int board_id_;
  std::string motor_status_prefix_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanRobomasRef>::SharedPtr sabacan_ref_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr status_sub_;
  std::array<rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr, 4> motor_pubs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToSabacanNode>());
  rclcpp::shutdown();
  return 0;
}
