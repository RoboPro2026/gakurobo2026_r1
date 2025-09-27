/**
 * @file r1_state_machine_node.cpp
 * @author Yamaguchi Yudai
 * @brief R1の状態遷移ノード
 * @version 0.1
 * @date 2025-09-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <variant>

#include "geometry_msgs/msg/twist.hpp"
#include "magic_enum.hpp"
#include "r1_state_machine/state_machine.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class R1StateMachineNode : public rclcpp::Node
{
public:
  R1StateMachineNode() : Node("r1_state_machine_node")
  {
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&R1StateMachineNode::joy_callback, this, std::placeholders::_1));

    timer_publisher_ =
      this->create_wall_timer(10ms, std::bind(&R1StateMachineNode::timer_callback, this));

    state_machine_ = std::make_shared<StateMachine>(
      [this](std::string msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); });
    state_machine_->set_next_state({MainState::MANUAL, ManualSubState::NONE});
  }

  // --- コールバック関数 ---
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    left_stick_x_ = msg->axes[0];
    left_stick_y_ = msg->axes[1];
    right_stick_x_ = msg->axes[3];
    right_stick_y_ = msg->axes[4];
  }

  void timer_callback(void)
  {
    // 状態を更新
    state_machine_->update();
    // タスクを実行
    main_task();
  }

  // --- 各状態のタスク ---
  void idle_task(void)
  {
    // 速度指令値を0にする
    target_vel_.linear.x = 0.0;
    target_vel_.linear.y = 0.0;
    target_vel_.angular.z = 0.0;
    cmd_vel_publisher_->publish(target_vel_);
  }

  void emergency_task(void)
  {
    // 速度指令値を0にする
    target_vel_.linear.x = 0.0;
    target_vel_.linear.y = 0.0;
    target_vel_.angular.z = 0.0;
    cmd_vel_publisher_->publish(target_vel_);
  }

  void manual_task(void)
  {
    // スティックの状態に応じて、足回りを動かす
    // TODO: 必要に応じて、符号の反転や係数をかける。
    target_vel_.linear.x = left_stick_x_;
    target_vel_.linear.y = left_stick_y_;
    target_vel_.angular.z = right_stick_x_;
    cmd_vel_publisher_->publish(target_vel_);
  }

  void auto_task(void) {}

  void main_task(void)
  {
    auto current_state = state_machine_->get_current_state();
    if (current_state.main == MainState::IDLE) {
      idle_task();
    } else if (current_state.main == MainState::EMERGENCY) {
      emergency_task();
    } else if (current_state.main == MainState::MANUAL) {
      manual_task();
    } else if (current_state.main == MainState::AUTO) {
      auto_task();
    }
  }

  std::shared_ptr<StateMachine> state_machine_;

  // publisherとsubscriber
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;

  // 速度指令値
  geometry_msgs::msg::Twist target_vel_;
  // joy_stick
  double left_stick_x_ = 0.0f;
  double left_stick_y_ = 0.0f;
  double right_stick_x_ = 0.0f;
  double right_stick_y_ = 0.0f;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1StateMachineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}