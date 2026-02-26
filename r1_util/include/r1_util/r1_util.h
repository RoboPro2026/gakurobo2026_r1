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

constexpr int ACT_N = 3;
constexpr int ACT_NONE = 0;
constexpr int ACT0_START = 1;
constexpr int ACT0_MOVE = 2;
constexpr int ACT0_ROTATE = 3;
constexpr int ACT0_FINISH = 4;
constexpr int ACT1_START = 11;
constexpr int ACT1 = 12;
constexpr int ACT1_FINISH = 13;
constexpr int ACT2_START = 21;
constexpr int ACT2 = 22;
constexpr int ACT2_FINISH = 23;

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
inline double angle_normalize(double angle)
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
inline double angle_diff(double current_angle, double prev_angle)
{
  std::complex<double> current = std::polar(1.0, current_angle);
  std::complex<double> prev = std::polar(1.0, prev_angle);
  std::complex<double> diff = current / prev;  // 位相差
  return std::arg(diff);
}
