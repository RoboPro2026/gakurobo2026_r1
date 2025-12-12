/**
 * @file util.h
 * @author Yamaguchi Yudai
 * @brief 
 * @version 0.1
 * @date 2025-12-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <cmath>
#include <complex>
#include <vector>

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
   * 一度複素数平面に変換することで、-piとpiの境界も簡単に計算することができる
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