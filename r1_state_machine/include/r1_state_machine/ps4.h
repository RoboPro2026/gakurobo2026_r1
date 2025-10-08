/**
 * @file ps4.h
 * @author Yamaguchi Yudai
 * @brief joyからps4の構造体に変換するプログラム
 * @note 正確にはps4じゃなくで、dualshock4だが、細かいことは気にしない。
 * @version 0.1
 * @date 2025-10-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

struct PS4Data
{
  double left_stick_x = 0.0;   // left stick left(1) / right(-1)
  double left_stick_y = 0.0;   // left stick up(1) / down(-1)
  double right_stick_x = 0.0;  // right stick left(1) / right(-1)
  double right_stick_y = 0.0;  // right stick up(1) / down(-1)
  double l2_analog = 0.0;      // L2 trigger(1) / not pressed(0)。joyとは異なるので注意
  double r2_analog = 0.0;      // R2 trigger(1) / not pressed(0)。joyとは異なるので注意
  bool left_stick_pushed = false;
  bool right_stick_pushed = false;
  bool up = false;
  bool down = false;
  bool left = false;
  bool right = false;
  bool cross = false;
  bool circle = false;
  bool triangle = false;
  bool square = false;
  bool l1 = false;
  bool r1 = false;
  bool l2 = false;
  bool r2 = false;
  bool l3 = false;
  bool r3 = false;
  bool ps = false;
};

class PS4
{
public:
  PS4Data data;

private:
  PS4Data prev_data;
  rclcpp::Time last_time_ = rclcpp::Clock().now();
  bool is_connected_ = false;
  bool prev_is_connected_ = false;
  sensor_msgs::msg::Joy::SharedPtr msg_;
  bool update_flag_ = false;
  double deadzone_ = 0.1;

  double apply_deadzone(double x) { return std::abs(x) >= deadzone_ ? x : 0.0; }

public:
  PS4() {}

  PS4(double deadzone)
  {
    if (0 <= deadzone && deadzone <= 1.0) {
      deadzone_ = deadzone;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("PS4"), "deadzone errror. deadzone = %f", deadzone);
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // 接続されているとみなす
    is_connected_ = true;
    last_time_ = rclcpp::Clock().now();
    msg_ = msg;
    update_flag_ = true;
  }

  /**
   * @brief 制御周期で呼び出す関数。
   * releasedやpushedを読み飛ばすことがないように、joyのデータをPS4Data構造体への代入も、この関数内で行う。
   * 
   */
  void update()
  {
    // 接続状況を更新
    prev_is_connected_ = is_connected_;
    if ((rclcpp::Clock().now() - last_time_).seconds() < 0.3) {
      is_connected_ = true;
    } else {
      is_connected_ = false;
    }
    // 接続状況が変化した場合は、ログを出力
    if (is_connected_ != prev_is_connected_) {
      if (is_connected_)
        RCLCPP_WARN(rclcpp::get_logger("PS4"), "PS4 connected");
      else
        RCLCPP_WARN(rclcpp::get_logger("PS4"), "PS4 disconnected");
    }

    if (is_connected_ && update_flag_) {
      // PS4のデータを更新
      prev_data = data;  // 前のデータを保存

      auto msg = msg_;
      data.left_stick_x = apply_deadzone(msg->axes[0]);
      data.left_stick_y = apply_deadzone(msg->axes[1]);
      data.right_stick_x = apply_deadzone(msg->axes[3]);
      data.right_stick_y = apply_deadzone(msg->axes[4]);
      // 0(押されていない)~1(押されている)に変換
      // TODO: 必要であれば、l2_analogとr2_analogにも不感帯を適応する
      data.l2_analog = (-msg->axes[2] + 1.0) / 2.0;  // 0~1
      data.r2_analog = (-msg->axes[5] + 1.0) / 2.0;  // 0~1
      data.left_stick_pushed = msg->buttons[11];
      data.right_stick_pushed = msg->buttons[12];
      // joyの実装が2つのボタンを1つのaxesで共有している関係で、同時押しには非対応
      data.up = msg->axes[7] == 1.0;
      data.down = msg->axes[7] == -1.0;
      data.left = msg->axes[6] == 1.0;
      data.right = msg->axes[6] == -1.0;
      data.cross = msg->buttons[0];
      data.circle = msg->buttons[1];
      data.triangle = msg->buttons[2];
      data.square = msg->buttons[3];
      data.l1 = msg->buttons[4];
      data.r1 = msg->buttons[5];
      data.l2 = msg->buttons[6];
      data.r2 = msg->buttons[7];
      data.l3 = msg->buttons[8];
      data.r3 = msg->buttons[9];
      data.ps = msg->buttons[10];
    } else if (!is_connected_) {
      // 未接続の場合はスティックのデータを0にする
      prev_data = data = PS4Data();
    }
    // update関数終了時にflagはfalseにする。
    update_flag_ = false;
  }

  void set_deadzone(double deadzone)
  {
    if (0 <= deadzone && 1 <= deadzone) {
      deadzone_ = deadzone;
    }
  }
  double get_deadzone(void) { return deadzone_; }
  bool is_connected() { return is_connected_; }

  // --- Left Stick ---
  bool is_pushed_left_stick() { return data.left_stick_pushed && !prev_data.left_stick_pushed; }
  bool is_released_left_stick() { return !data.left_stick_pushed && prev_data.left_stick_pushed; }

  // --- Right Stick ---
  bool is_pushed_right_stick() { return data.right_stick_pushed && !prev_data.right_stick_pushed; }
  bool is_released_right_stick()
  {
    return !data.right_stick_pushed && prev_data.right_stick_pushed;
  }

  // --- D-Pad (Directional Buttons) ---
  bool is_pushed_up() { return data.up && !prev_data.up; }
  bool is_released_up() { return !data.up && prev_data.up; }
  bool is_pushed_down() { return data.down && !prev_data.down; }
  bool is_released_down() { return !data.down && prev_data.down; }
  bool is_pushed_left() { return data.left && !prev_data.left; }
  bool is_released_left() { return !data.left && prev_data.left; }
  bool is_pushed_right() { return data.right && !prev_data.right; }
  bool is_released_right() { return !data.right && prev_data.right; }

  // --- Action Buttons (Cross, Circle, Triangle, Square) ---
  bool is_pushed_cross() { return data.cross && !prev_data.cross; }
  bool is_released_cross() { return !data.cross && prev_data.cross; }
  bool is_pushed_circle() { return data.circle && !prev_data.circle; }
  bool is_released_circle() { return !data.circle && prev_data.circle; }
  bool is_pushed_triangle() { return data.triangle && !prev_data.triangle; }
  bool is_released_triangle() { return !data.triangle && prev_data.triangle; }
  bool is_pushed_square() { return data.square && !prev_data.square; }
  bool is_released_square() { return !data.square && prev_data.square; }

  // --- Shoulder Buttons (L1, R1) ---
  bool is_pushed_l1() { return data.l1 && !prev_data.l1; }
  bool is_released_l1() { return !data.l1 && prev_data.l1; }
  bool is_pushed_r1() { return data.r1 && !prev_data.r1; }
  bool is_released_r1() { return !data.r1 && prev_data.r1; }

  // --- Trigger Buttons (L2, R2) ---
  bool is_pushed_l2() { return data.l2 && !prev_data.l2; }
  bool is_released_l2() { return !data.l2 && prev_data.l2; }
  bool is_pushed_r2() { return data.r2 && !prev_data.r2; }
  bool is_released_r2() { return !data.r2 && prev_data.r2; }

  // --- Stick Press Buttons (L3, R3) ---
  bool is_pushed_l3() { return data.l3 && !prev_data.l3; }
  bool is_released_l3() { return !data.l3 && prev_data.l3; }
  bool is_pushed_r3() { return data.r3 && !prev_data.r3; }
  bool is_released_r3() { return !data.r3 && prev_data.r3; }

  // --- PS Button ---
  bool is_pushed_ps() { return data.ps && !prev_data.ps; }
  bool is_released_ps() { return !data.ps && prev_data.ps; }
};