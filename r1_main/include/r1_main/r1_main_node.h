/**
 * @file r1_main_node.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-18
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <variant>

#include "geometry_msgs/msg/twist.hpp"
#include "magic_enum.hpp"
#include "r1_main/ps4.h"
#include "r1_main/simple_trapezoid.h"
#include "r1_main/state_machine.h"
#include "r1_msgs/msg/angle_motion.hpp"
#include "r1_msgs/msg/gpio_esc_ref.hpp"
#include "r1_msgs/msg/gpio_input.hpp"
#include "r1_msgs/msg/gpio_pwm_ref.hpp"
#include "r1_msgs/msg/gpio_servo_ref.hpp"
#include "r1_msgs/msg/linear_motion.hpp"
#include "r1_msgs/msg/motor_ref.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

class R1MainNode : public rclcpp::Node
{
public:
  std::shared_ptr<StateMachine> state_machine_;
  std::shared_ptr<PS4> ps4_;
  SimpleTrapezoid simple_trapezoid_vx_;
  SimpleTrapezoid simple_trapezoid_vy_;
  SimpleTrapezoid simple_trapezoid_omega_;

  // publisherとsubscriber
  // 足回りの速度指令
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // joyの受信
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

  // KFS回収
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_fx_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_fz_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_fyaw_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_rx_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_rz_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_ryaw_position_ref_publisher_;
  // 展開補助
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_expand_assist_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_expand_assist_position_ref_publisher_;
  // R2昇降
  rclcpp::Publisher<r1_msgs::msg::MotorRef>::SharedPtr r2_lift_motor_ref_publisher_;
  // KFS真空ポンプ
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_front_pump_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_rear_pump_gpio_pwm_ref_publisher_;
  // 真空電磁弁
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_front_valve_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_rear_valve_gpio_pwm_ref_publisher_;
  // KFSリミットスイッチ
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr kfs_front_switch_status_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr kfs_rear_switch_status_subscription_;
  // TODO: detect_originは後で書く、また現在の動作状況についても出力するようにする

  rclcpp::TimerBase::SharedPtr timer_publisher_;

  // 速度指令値
  geometry_msgs::msg::Twist target_vel_;

  bool kfs_front_switch_status_ = false;
  bool kfs_rear_switch_status_ = false;

  R1MainNode();

  // ========== コールバック関数 =========
  void kfs_front_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  void kfs_rear_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback(void);
  // ========== 各動作の関数 ==========
  void chassis_move_vel(double vx, double vy, double omega);
  void kfs_fx(double pos);
  void kfs_fz(double pos);
  void kfs_fyaw(double pos);
  void kfs_rx(double pos);
  void kfs_rz(double pos);
  void kfs_ryaw(double pos);
  void front_expand_assist(double pos);
  void rear_expand_assist(double pos);
  void r2_lift(double vel);
  void kfs_front_pump(double pwm);
  void kfs_rear_pump(double pwm);
  void kfs_front_valve(bool on);
  void kfs_rear_valve(bool on);
  // ========== センサーの取得 ==========
  bool get_kfs_front_switch_status(void) { return kfs_front_switch_status_; }
  bool get_kfs_rear_switch_status(void) { return kfs_rear_switch_status_; }
  // ========== 各状態のタスク ==========
  void idle_task(void);
  void emergency_task(void);
  void manual_task(void);
  void auto_task(void);
  void main_task(void);
};