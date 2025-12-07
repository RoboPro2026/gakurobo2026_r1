/**
 * @file trapezoid_designer.h
 * @author Yamaguchi Yudai
 * @brief S次台形制御のプログラム
 * @version 0.1
 * @date 2025-12-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "r1_control/util.h"
#include "rclcpp/rclcpp.hpp"

class TrapezoidDesigner
{
public:
  TrapezoidDesigner() {}
  /**
   * @brief 
   * 
   * @param d 走行距離
   * @param v_s 始点速度
   * @param v_e 終点速度
   * @param a_max 最大加速度
   * @param v_max 最大跳度
   */
  void calc(double d, double v_s, double v_e, double a_max, double j_max)
  {
    d_ = d;
    v_s_ = v_s;
    v_e_ = v_e;
    a_m_ = sign(v_e_ - v_s_) * abs(a_max);
    j_m_ = sign(v_e_ - v_s_) * abs(j_max);
    tc_ = abs(a_max / j_max);
    tm_ = abs((v_e_ - v_s_) / a_max) - tc_;
    if (tm_ > 0.0) {
      // 時間
      t0_ = 0.0;
      t1_ = t0_ + tc_;
      t2_ = t1_ + tm_;
      t3_ = t2_ + tc_;
      // 速度
      v0_ = v_s_;
      v1_ = v0_ + 1.0 / 2.0 * j_m_ * tc_ * tc_;
      v2_ = v1_ + a_m_ * tm_;
      v3_ = v_e_;
      // 位置
      x0_ = 0.0;
      x1_ = x0_ + v0_ * tc_ + j_m_ * pow(tc_, 3);
      x2_ = x1 + v1_ * tm_;
      x3_ = x0_ + 1.0 / 2.0 * (v0_ + v3_) * (2 * tc_ + tm_);
    } else {
      // tm <= 0

      // 時間
      double tcp = sqrt(abs((v_e_ - v_s_) / j_m_));
      tc_ = tcp;
      t0_ = 0.0;
      t1_ = t0_ + tc_;
      t2_ = t1_;
      t3_ = t2_ + tc_;
      // 速度
      v0_ = v_s_;
      v1_ = v0_ + 1.0 / 2.0 * (v_s_ + v_e_);
      v2_ = v1_;
      v3_ = v_e_;
      // 位置
      x0_ = 0.0;
      x1_ = x0_ + v1_ * tcp + 1.0 / 6.0 * j_m_ * pow(tcp, 3);
      x2_ = x1_;
      x3_ = x0_ + 2 * v1_ * tcp;
    }
  }

  /**
   * @brief 時刻tのときの躍度を取得する
   * 
   * @param t 
   * @return double 
   */
  double get_j(double t)
  {
    if (t <= t0_) {
      return 0.0;
    } else if (t <= t1_) {
      return j_m_;
    } else if (t <= t2_) {
      return 0.0;
    } else if (t <= t3_) {
      return -j_m_;
    } else {
      return 0.0;
    }
  }

  /**
   * @brief 時刻tのときの加速度を取得する
   * 
   * @param t 
   * @return double 
   */
  double get_a(double t)
  {
    if (t <= t0_) {
      return 0.0;
    } else if (t <= t1_) {
      return j_m_ * (t - t0_);
    } else if (t <= t2_) {
      return a_m_;
    } else if (t <= t3_) {
      return -j_m_ * (t - t3_);
    } else {
      return 0.0;
    }
  }

  /**
   * @brief 時刻tのときの速度を取得する
   * 
   * @param t 
   * @return double 
   */
  double get_v(double t)
  {
    if (t <= t0_) {
      return v0_;
    } else if (t <= t1_) {
      return v0_ + 1.0 / 2.0 * (t - t0_) * (t - t0_);
    } else if (t <= t2_) {
      return v1_ + a_m_ * (t - t1_);
    } else if (t <= t3_) {
      return v3_ - 1.0 / 2.0 * j_m_ * (t - t3_) * (t - t3_);
    } else {
      return v3_;
    }
  }

  /**
   * @brief 時刻tのときの位置を取得する
   * 
   * @param t 
   * @return double 
   */
  double get_x(double t)
  {
    if (t <= t0_) {
      return x0_ + v0_ * (t - t0_);
    } else if (t <= t1_) {
      return x0_ + v0_ * (t - t0_) + 1.0 / 6.0 * j_m_ * pow(t - t0_, 3);
    } else if (t <= t2_) {
      return x1_ + v1_ * (t - t1_) + 1.0 / 2.0 * a_m_ * (t - t1_) * (t - t1_);
    } else if (t <= t3_) {
      return x3_ + v3_ * (t - t3_) - 1.0 / 6.0 * pow(t - t3_, 3);
    } else {
      return x3_ + v3_ * (t - t3_);
    }
  }

private:
  /**
   * @brief xの符号を返す
   * 
   * @param x 
   * @return double xが0.0以上なら1.0、負なら-1.0を返す
   */
  double sign(double x) { return (x >= 0) ? 1.0 : -1.0; }

  double t0_, t1_, t2_, t3_;
  double x0_, x1_, x2_, x3_;
  double v0_, v1_, v2_, v3_;
  double a0_, a1_, a2_, a3_;
  double d_, v_s_, v_e_, a_m_, j_m_, tm_, tc_;
};