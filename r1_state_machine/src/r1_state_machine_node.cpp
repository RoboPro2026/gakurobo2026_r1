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
#include "r1_state_machine/ps4.h"
#include "r1_state_machine/simple_trapezoid.h"
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

    simple_trapezoid_vx_ = SimpleTrapezoid(3.0, 0.01);  // 加速度1.0m/s^2、制御周期10ms
    simple_trapezoid_vy_ = SimpleTrapezoid(3.0, 0.01);
    simple_trapezoid_omega_ = SimpleTrapezoid(3.0, 0.01);

    ps4_ = std::make_shared<PS4>("PS4");

    state_machine_ = std::make_shared<StateMachine>();
    state_machine_->set_next_state({MainState::MANUAL, ManualSubState::NONE});
  }

  // --- コールバック関数 ---
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) { ps4_->joy_callback(msg); }

  void timer_callback(void)
  {
    ps4_->update();
    // 状態を更新
    state_machine_->update();
    // タスクを実行
    // ps4_->print_data();
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
    if (ps4_->is_connected()) {
      // スティックの状態に応じて、足回りを動かす
      double VEL = 1.0;
      if (ps4_->data.right) {
        target_vel_.linear.x = -VEL;
        target_vel_.linear.y = 0;
        target_vel_.angular.z = 0;
      } else if (ps4_->data.left) {
        target_vel_.linear.x = VEL;
        target_vel_.linear.y = 0;
        target_vel_.angular.z = 0;
      } else if (ps4_->data.up) {
        target_vel_.linear.x = 0;
        target_vel_.linear.y = VEL;
        target_vel_.angular.z = 0;
      } else if (ps4_->data.down) {
        target_vel_.linear.x = 0;
        target_vel_.linear.y = -VEL;
        target_vel_.angular.z = 0;
      } else {
        // TODO: 必要に応じて、符号の反転や係数をかける。
        double stick_max_velocity = 3.0;
        double vx_ref = stick_max_velocity * ps4_->data.left_stick_x;
        double vy_ref = stick_max_velocity * ps4_->data.left_stick_y;
        double vz_ref = ps4_->data.right_stick_x;
        // 台形制御で速度を滑らかに変化させる
        target_vel_.linear.x = simple_trapezoid_vx_.update(vx_ref);
        target_vel_.linear.y = simple_trapezoid_vy_.update(vy_ref);
        target_vel_.angular.z = simple_trapezoid_omega_.update(vz_ref);
        // RCLCPP_INFO(this->get_logger(), "vx: %.2f, vx_ref: %.2f", target_vel_.linear.x, vx_ref);
        // target_vel_.linear.x = stick_max_velocity * ps4_->data.left_stick_x;
        // target_vel_.linear.y = stick_max_velocity * ps4_->data.left_stick_y;
        // target_vel_.angular.z = ps4_->data.right_stick_x;
      }
    } else {
      target_vel_.linear.x = 0.0;
      target_vel_.linear.y = 0.0;
      target_vel_.angular.z = 0.0;
    }
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

  // なんとなくshared_ptr。特に理由はない。
  std::shared_ptr<StateMachine> state_machine_;
  std::shared_ptr<PS4> ps4_;
  SimpleTrapezoid simple_trapezoid_vx_;
  SimpleTrapezoid simple_trapezoid_vy_;
  SimpleTrapezoid simple_trapezoid_omega_;

  // publisherとsubscriber
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;

  // 速度指令値
  geometry_msgs::msg::Twist target_vel_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1StateMachineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}