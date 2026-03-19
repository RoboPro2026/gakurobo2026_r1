/**
 * @file r1_util.h
 * @author Yudai Yamaguchi (yudai.yy0804@gmail.com)
 * @brief R1の共通ライブラリ
 * @version 0.1
 * @date 2026-02-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>
#include <complex>

#include "rclcpp/rclcpp.hpp"

constexpr int FOREST_N = 12;
constexpr int ACT_N = 4;

enum class ChassisAct
{
  NONE = 0,
  ACT0_START = 1,
  ACT0 = 2,
  ACT0_FINISH = 3,
  ACT1_START = 11,
  ACT1 = 12,
  ACT1_FINISH = 13,
  ACT2_START = 21,
  ACT2 = 22,
  ACT2_FINISH = 23,
  ACT3_START = 31,
  ACT3 = 32,
  ACT3_FINISH = 33
};

/**
 * @brief 1次のローパスフィルタ
 * 
 * @param x 現在の入力
 * @param prev_y 限界の出力
 * @param tau 時定数[s]
 * @param dt 制御周期[s]
 * @return double 出力 y = alpha * x + (1 - alpha) * prev_y
 * ただし alpha = dt / (tau + dt)
 */
double lpf(double x, double prev_y, double tau, double dt)
{
  double alpha = dt / (tau + dt);
  return alpha * x + (1 - alpha) * prev_y;
}

/**
 * @brief 指定した開始点から終了点までの範囲を、指定した数だけ等間隔に分割して配列を生成する
 * 
 * @param start 開始点
 * @param end 終了点
 * @param num 分割数
 * @return std::vector<double> 
 */
std::vector<double> linspace(double start, double end, int num)
{
  std::vector<double> x(num);
  double step = (end - start) / (num - 1);
  for (int i = 0; i < num; i++) {
    x[i] = start + step * i;
  }
  return x;
}

/**
   * @brief xの符号を返す
   * 
   * @param x 
   * @return double xが0.0以上なら1.0、負なら-1.0を返す
   */
double sign(double x) { return (x >= 0) ? 1.0 : -1.0; }

/**
   * @brief 角度を-pi~piの範囲に正規化する
   * 
   * @param angle 
   * @return double 
   */
double angle_normalize(double angle)
{
  std::complex<double> ret = std::polar(1.0, angle);
  return std::arg(ret);
}

/**
   * @brief 角度差を計算する。計算結果は-pi~pi
   * 
   * @param current_angle 
   * @param prev_angle 
   * @return double 
   */
double angle_diff(double current_angle, double prev_angle)
{
  std::complex<double> current = std::polar(1.0, current_angle);
  std::complex<double> prev = std::polar(1.0, prev_angle);
  std::complex<double> diff = current / prev;  // 位相差
  return std::arg(diff);
}

/**
 * @brief 現在の位置が、指定した開始点と終了点の範囲内にあるかどうかを判定する
 * 
 * @param current_x 
 * @param current_y 
 * @param x1 
 * @param y1 
 * @param x2 
 * @param y2 
 * @return true 
 * @return false 
 */
bool is_within_range(double current_x, double current_y, double x1, double y1, double x2, double y2)
{
  // x2とy2がx1とy1より大きくなるようにする。
  if (x1 > x2) {
    std::swap(x1, x2);
  }
  if (y1 > y2) {
    std::swap(y1, y2);
  }
  bool is_x_in_range = (x1 <= current_x && current_x <= x2);
  bool is_y_in_range = (y1 <= current_y && current_y <= y2);
  return is_x_in_range && is_y_in_range;
}