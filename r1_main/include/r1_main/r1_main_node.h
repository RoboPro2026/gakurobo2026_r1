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
#include "std_msgs/msg/int32.hpp"

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

  // ========== KFS回収 ==========
  // 指令値Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_fx_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_fz_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_fyaw_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_rx_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_rz_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_ryaw_position_ref_publisher_;
  // 原点検出Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_fx_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_fz_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_fyaw_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_rx_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_rz_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_ryaw_detect_origin_publisher_;
  // mode Subscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_fx_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_fz_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_fyaw_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_rx_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_rz_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_ryaw_mode_status_subscription_;
  // ========== 展開 ==========
  // 指令値Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_expand_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_expand_position_ref_publisher_;
  // 原点検出Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_expand_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rear_expand_detect_origin_publisher_;
  // mode Subscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr front_expand_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rear_expand_mode_status_subscription_;
  // ========== R2昇降 ==========
  rclcpp::Publisher<r1_msgs::msg::MotorRef>::SharedPtr r2_lift_motor_ref_publisher_;
  // ========== ポール回収 ==========
  // 指令値Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pole_x_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pole_y_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pole_roger_position_ref_publisher_;
  // 原点検出Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pole_x_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pole_y_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pole_roger_detect_origin_publisher_;
  // mode Subscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pole_x_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pole_y_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pole_roger_mode_status_subscription_;
  // ========== やり ==========
  // 指令値Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spear_roger1_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spear_roger2_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spear_move_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spear_rotate_position_ref_publisher_;
  // 原点検出Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_roger1_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_roger2_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_move_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_rotate_detect_origin_publisher_;
  // mode Subscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spear_roger1_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spear_roger2_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spear_move_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spear_rotate_mode_status_subscription_;

  // KFS真空ポンプ
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_front_pump_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_rear_pump_gpio_pwm_ref_publisher_;
  // 真空電磁弁
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_front_valve_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_rear_valve_gpio_pwm_ref_publisher_;
  // KFSリミットスイッチ
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr kfs_front_switch_status_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr kfs_rear_switch_status_subscription_;
  // ポール回収サーボ
  rclcpp::Publisher<r1_msgs::msg::GpioServoRef>::SharedPtr pole_servo1_gpio_servo_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioServoRef>::SharedPtr pole_servo2_gpio_servo_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioServoRef>::SharedPtr pole_servo3_gpio_servo_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioServoRef>::SharedPtr pole_servo4_gpio_servo_ref_publisher_;
  //  ポール回収電磁弁
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve1_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve2_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve3_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve4_gpio_pwm_ref_publisher_;
  // やりハンド電磁弁
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr spear_hand_valve_gpio_pwm_ref_publisher_;
  // やりリミットスイッチ
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr spear_move_switch_status_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr spear_rotate_switch_status_subscription_;
  // タイマー
  rclcpp::TimerBase::SharedPtr timer_publisher_;

  // 速度指令値
  geometry_msgs::msg::Twist target_vel_;

  bool kfs_front_switch_status_ = false;
  bool kfs_rear_switch_status_ = false;
  bool spear_move_switch_status_ = false;
  bool spear_rotate_switch_status_ = false;

  bool is_kfs_fx_pos_mode_ = false;
  bool is_kfs_fz_pos_mode_ = false;
  bool is_kfs_fyaw_pos_mode_ = false;
  bool is_kfs_rx_pos_mode_ = false;
  bool is_kfs_rz_pos_mode_ = false;
  bool is_kfs_ryaw_pos_mode_ = false;
  bool is_front_expand_pos_mode_ = false;
  bool is_rear_expand_pos_mode_ = false;
  bool is_pole_x_pos_mode_ = false;
  bool is_pole_y_pos_mode_ = false;
  bool is_pole_roger_pos_mode_ = false;
  bool is_spear_roger1_pos_mode_ = false;
  bool is_spear_roger2_pos_mode_ = false;
  bool is_spear_move_pos_mode_ = false;
  bool is_spear_rotate_pos_mode_ = false;

  R1MainNode();

  // ========== コールバック関数 =========
  // modeのcalalback
  void kfs_fx_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_fz_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_fyaw_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_rx_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_rz_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_ryaw_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void front_expand_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void rear_expand_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void pole_x_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void pole_y_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void pole_roger_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void spear_roger1_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void spear_roger2_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void spear_move_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void spear_rotate_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  // スイッチのcallback
  void kfs_front_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  void kfs_rear_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  void spear_move_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  void spear_rotate_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  // joyのcallback
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback(void);
  // ========== 各動作の関数 ==========
  // 足回り
  void chassis_move_vel(double vx, double vy, double omega);
  // KFS回収
  void kfs_fx(double pos);
  void kfs_fz(double pos);
  void kfs_fyaw(double pos);
  void kfs_rx(double pos);
  void kfs_rz(double pos);
  void kfs_ryaw(double pos);
  // 展開
  void front_expand(double pos);
  void rear_expand(double pos);
  // R2昇降
  void r2_lift(double vel);
  // ポール回収
  void pole_x(double pos);
  void pole_y(double pos);
  void pole_roger(double pos);
  // やり
  void spear_roger1(double pos);
  void spear_roger2(double pos);
  void spear_move(double pos);
  void spear_rotate(double pos);
  // KFS真空ポンプ・電磁弁
  void kfs_front_pump(double pwm);
  void kfs_rear_pump(double pwm);
  void kfs_front_valve(bool on);
  void kfs_rear_valve(bool on);
  // ポール真空ポンプ・電磁弁
  void pole_servo1(int angle);
  void pole_servo2(int angle);
  void pole_servo3(int angle);
  void pole_servo4(int angle);
  void pole_valve1(bool on);
  void pole_valve2(bool on);
  void pole_valve3(bool on);
  void pole_valve4(bool on);
  // やり
  void spear_hand_valve(bool on);
  // ========== 原点検出関数 ==========
  // KFS回収
  void kfs_fx_detect_origin(void);
  void kfs_fz_detect_origin(void);
  void kfs_fyaw_detect_origin(void);
  void kfs_rx_detect_origin(void);
  void kfs_rz_detect_origin(void);
  void kfs_ryaw_detect_origin(void);
  // 展開
  void front_expand_detect_origin(void);
  void rear_expand_detect_origin(void);
  // ポール回収
  void pole_x_detect_origin(void);
  void pole_y_detect_origin(void);
  void pole_roger_detect_origin(void);
  // やり
  void spear_roger1_detect_origin(void);
  void spear_roger2_detect_origin(void);
  void spear_move_detect_origin(void);
  void spear_rotate_detect_origin(void);
  // ========== センサーの取得 ==========
  bool get_kfs_front_switch_status(void) { return kfs_front_switch_status_; }
  bool get_kfs_rear_switch_status(void) { return kfs_rear_switch_status_; }
  bool get_spear_move_switch_status(void) { return spear_move_switch_status_; }
  bool get_spear_rotate_switch_status(void) { return spear_rotate_switch_status_; }
  // ========== 各状態のタスク ==========
  void idle_task(void);
  void emergency_task(void);
  void manual_task(void);
  void auto_task(void);
  void main_task(void);
  // ========== テスト関数 ==========
  void test_front_kfs(void);
  void test_rear_kfs(void);
  void test_pole(void);
  void test_spear(void);
  void test_r2_lift(void);
};
