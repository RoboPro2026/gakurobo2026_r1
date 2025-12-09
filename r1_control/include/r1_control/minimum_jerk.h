/**
 * @file minimum_jerk.h
 * @author Yudai Yamaguchi
 * @brief 5次の最小躍度曲線
 * @ref https://www.jneurosci.org/content/jneuro/5/7/1688.full.pdf
 * @version 0.1
 * @date 2025-12-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "r1_control/util.h"
#include "rclcpp/rclcpp.hpp"

class MinimumJerk
{
public:
  MinimumJerk() {}

  /**
   * @brief Construct a new Minimum Jerk object
   * 
   * @param x0 開始位置
   * @param xf 終了位置
   * @param ts 開始時間
   * @param tf 終了時間
   */
  MinimumJerk(double x0, double xf, double ts, double tf) : x0_(x0), xf_(xf), ts_(ts), tf_(tf) {}

  /**
   * @brief パラメータを設定する
   * 
   * @param x0 開始位置
   * @param xf 終了位置
   * @param ts 開始時間
   * @param tf 終了時間
   */
  void setParam(double x0, double xf, double ts, double tf)
  {
    x0_ = x0;
    xf_ = xf;
    ts_ = ts;
    tf_ = tf;
  }

  /**
   * @brief x 位置を計算する
   * x(t) = x0 + (xf - x0) * (10*tau^3 - 15*tau^4 + 6*tau^5)
   * ただし、tau = (t - ts) / tf
   * 
   * @param t 現在時刻
   * @return double 
   */
  double x(double t)
  {
    double tau = calc_tau(t);
    return x0_ + (xf_ - x0_) * (10.0 * pow(tau, 3) - 15.0 * pow(tau, 4) + 6.0 * pow(tau, 5));
  }

  /**
   * @brief v 速度を計算する(dx/dt)
   * 
   * @param t 現在時刻
   * @return double 
   */
  double v(double t)
  {
    double tau = calc_tau(t);
    return (xf_ - x0_) * (30.0 * pow(tau, 2) - 60.0 * pow(tau, 3) + 30.0 * pow(tau, 4)) / tf_;
  }

  /**
   * @brief a 加速度を計算する(d2x/dt2)
   * 
   * @param t 現在時刻
   * @return double 
   */
  double a(double t)
  {
    double tau = calc_tau(t);
    return (xf_ - x0_) * (60.0 * tau - 180.0 * pow(tau, 2) + 120.0 * pow(tau, 3)) / (pow(tf_, 2));
  }

  /**
   * @brief j 躍度を計算する(d3x/dt3)
   * 
   * @param t 現在時刻
   * @return double 
   */
  double j(double t)
  {
    double tau = calc_tau(t);
    return (xf_ - x0_) * (60.0 - 360.0 * tau + 360.0 * pow(tau, 2)) / (pow(tf_, 3));
  }

private:
  /**
   * @brief tau=(t - ts) / tf を計算する
   * 
   * @param t 現在時刻
   * @return double tauは0.0~1.0の範囲
   */
  double calc_tau(double t)
  {
    if (t < ts_)
      t = ts_;
    else if (t > tf_)
      t = tf_;
    return (t - ts_) / tf_;
  }

  double x0_;
  double xf_;
  double ts_;
  double tf_;
};
