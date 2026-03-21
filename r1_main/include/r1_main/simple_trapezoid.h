/**
 * @file simple_trapezoid.h
 * @author Yudai Yamaguchi
 * @brief 台形制御のプログラム
 * @version 0.1
 * @date 2025-12-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "rclcpp/rclcpp.hpp"

/**
 * @brief 距離の制約条件のない台形制御を計算する
 * 
 */
class SimpleTrapezoid
{
private:
  // 加速度
  double a_;
  // 制御周期
  double dt_;
  // 現在の速度
  double v_;
  // 1ステップの操作量
  double v_step_;

public:
  SimpleTrapezoid() { v_ = 0.0; }

  /**
   * @brief Construct a new Simple Trapezoid object
   * 
   * @param a 加速度[m/s^2]
   * @param dt 制御周期[sec]
   */
  SimpleTrapezoid(double a, double dt)
  {
    this->set_param(a, dt);
    v_ = 0.0;
  }

  /**
   * @brief Set the param object
   * 
   * @param a 加速度[m/s^2]
   * @param dt 制御周期[sec]
   */
  void set_param(double a, double dt)
  {
    a_ = a;
    dt_ = dt;
    v_step_ = a_ * dt_;
    // いらないログなのでコメントアウト
    // RCLCPP_INFO(
    //   rclcpp::get_logger("SimpleTrapezoid"), "a: %.2f, dt: %.4f, v_step: %.4f", a_, dt_, v_step_);
  }

  /**
   * @brief 速度をリセットする
   * 
   */
  void reset_v() { v_ = 0.0; }

  /**
   * @brief 台形制御の計算を行う
   * 
   * @param v_ref 速度指令値
   * @return double 
   */
  double update(double v_ref)
  {
    if (v_ref - v_step_ <= v_ && v_ <= v_ref + v_step_) {
      v_ = v_ref;
    } else if (v_ < v_ref) {
      v_ += v_step_;
    } else if (v_ > v_ref) {
      v_ -= v_step_;
    }
    return v_;
  }
  double get_v() { return v_; }
};
