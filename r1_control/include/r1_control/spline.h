/**
 * @file spline.h
 * @author Yamaguchi Yudai
 * @brief 3次スプライン補完を計算する
 * @ref https://www5d.biglobe.ne.jp/stssk/maze/spline.html
 * @version 0.1
 * @date 2025-12-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "r1_control/util.h"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 1次元の3次スプライン補間を計算するクラス
 * 
 */
class Spline
{
public:
  Spline() {}

  /**
   * @brief スプライン補間の計算を行う
   * 
   * @param _x 通過したいx座標の配列
   * @param _y 通過したいy座標の配列
   */
  void calc(std::vector<double> _x, std::vector<double> _y)
  {
    x_ = _x;
    y_ = _y;

    if (x_.size() != y_.size() || x_.size() <= 3) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid input data for spline calculation.");
      return;
    }
    // 変数を宣言
    n_ = x_.size();
    a_.resize(n_);
    b_.resize(n_);
    c_.resize(n_);
    d_.resize(n_);
    h_.resize(n_ - 1);

    // 係数aを代入
    a_ = y_;
    // 区間の幅hを計算
    for (int i = 0; i < n_ - 1; i++) {
      h_[i] = x_[i + 1] - x_[i];
    }

    // 連立方程式を計算
    // TODO:工夫すれば、対称行列になって高速化できるっぽいので、気が向いたらやる
    Eigen::MatrixXd A(n_, n_);
    Eigen::VectorXd b(n_);
    A.setZero();
    b.setZero();

    A(0, 0) = 1.0;
    A(n_ - 1, n_ - 1) = 1.0;
    for (int i = 1; i < n_ - 1; i++) {
      // i行目の値を更新
      A(i, i - 1) = h_[i - 1];
      A(i, i) = 2.0 * (h_[i - 1] + h_[i]);
      A(i, i + 1) = h_[i];
      b(i) = 3.0 * ((a_[i + 1] - a_[i]) / h_[i] - (a_[i] - a_[i - 1]) / h_[i - 1]);
    }

    // solve
    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

    // 係数cを代入
    for (int i = 0; i < n_; i++) {
      c_[i] = x(i);
    }
    // 係数bとdを計算
    for (int i = 1; i < n_; i++) {
      b_[i - 1] = (a_[i] - a_[i - 1]) / h_[i - 1] - h_[i - 1] * (2.0 * c_[i - 1] + c_[i]) / 3.0;
      d_[i - 1] = (c_[i] - c_[i - 1]) / (3.0 * h_[i - 1]);
    }
  }

  /**
   * @brief スプライン補間で位置を計算する
   * @param x 入力
   * @return 補完した位置
   */
  double get_y(double x)
  {
    // 入力値が範囲外の場合は、端の値を返す
    int i = get_index(x);
    if (i == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Input t is out of range in get_pos(): %f", x);
      return x < x_.front() ? y_.front() : y_.back();
    }
    double dx = x - x_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
  }

  /**
   * @brief 1階微分を計算する
   * 
   * @param x 
   * @return double 
   */
  double get_dy(double x)
  {
    int i = get_index(x);
    if (i == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Input t is out of range in get_pos(): %f", x);
      return 0.0;
    }

    double dx = x - x_[i];
    return b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
  }

  /**
   * @brief 2階微分を計算する
   * 
   * @param x 
   * @return double 
   */
  double get_d2y(double x)
  {
    int i = get_index(x);
    if (i == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Input t is out of range in get_pos(): %f", x);
      return 0.0;
    }

    double dx = x - x_[i];
    return 2.0 * c_[i] + 6.0 * d_[i] * dx;
  }

  /**
   * @brief 曲率を計算する
   * @param x 
   * @return 曲率
   */
  double get_curvature(double x)
  {
    int i = get_index(x);
    if (i == -1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Input t is out of range in get_curvature(): %f", x);
      return 0.0;
    }
    // 1階微分
    double dy = get_dy(x);
    // 2階微分
    double d2y = get_d2y(x);
    return d2y / pow(1.0 + dy * dy, 1.5);
  }

  /**
   * @brief 入力値がどの区間に入るかを計算する
   * @param x 入力
   * @return 区間のインデックス
   */
  int get_index(double x)
  {
    // 動作例
    // x_ = {1, 3, 5, 7}, x = 4 →
    // 4 は区間 [3,5] に入る → index=1

    // 値が範囲外の場合は-1を返す
    if (x < x_.front() || x > x_.back()) {
      return -1;
    }
    auto it = std::lower_bound(x_.begin(), x_.end(), x);
    int index = std::max(int(it - x_.begin() - 1), 0);

    return index;
  }

private:
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
  std::vector<double> h_;
  int n_;
  double path_length_;
};

/**
 * @brief 2次元の3次スプライン補間を計算するクラス
 * 
 */
class Spline2D
{
public:
  Spline2D() {}

  /**
   * @brief スプライン補間の計算を行う
   * 
   * @param x 通過したいx座標の配列
   * @param y 通過したいy座標の配列
   */
  void calc(const std::vector<double> & x, const std::vector<double> & y)
  {
    x_ = x;
    y_ = y;
    t_.resize(x_.size());

    // 媒介変数tを計算。waypoint間の距離を計算し、最後に正規化して0~1の範囲にする。
    t_[0] = 0.0;
    for (size_t i = 1; i < x_.size(); i++) {
      double dx = x_[i] - x_[i - 1];
      double dy = y_[i] - y_[i - 1];
      double dist = hypot(dx, dy);
      t_[i] = t_[i - 1] + dist;
    }

    // t を 0.0 ~ 1.0 に正規化 (オプションですが、後続の処理を変えないために推奨)
    double total_length = t_.back();
    for (size_t i = 0; i < t_.size(); i++) {
      t_[i] /= total_length;
    }

    if (x_.size() != y_.size() || x_.size() <= 3) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid input data for spline calculation.");
      return;
    }

    sx_.calc(t_, x_);
    sy_.calc(t_, y_);
  }

  /**
   * @brief 位置を取得する
   * 
   * @param t 媒介変数(0.0 ~ 1.0)
   * @return std::pair<double, double> 
   */
  std::pair<double, double> get_pos(double t)
  {
    if (t < 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Input t is out of range in get_pos(): %f", t);
      return std::make_pair(x_.front(), y_.front());
    } else if (t > 1.0) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Input t is out of range in get_pos(): %f", t);
      return std::make_pair(x_.back(), y_.back());
    }
    return std::make_pair(sx_.get_y(t), sy_.get_y(t));
  }

  /**
   * @brief 2次元の曲率を取得する
   * 
   * @param t 媒介変数(0.0 ~ 1.0)
   * @return double 
   */
  double get_curvature(double t)
  {
    if (t < 0.0 || t > 1.0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Input t is out of range in get_curvature(): %f", t);
      return 0.0;
    }
    double dx = sx_.get_dy(t);
    double dy = sy_.get_dy(t);
    double d2x = sx_.get_d2y(t);
    double d2y = sy_.get_d2y(t);
    return (d2y * dx - d2x * dy) / std::pow(dx * dx + dy * dy, 1.5);
  }

  /**
   * @brief 各waypointに対応する媒介変数tを取得する
   * 
   * @return std::vector<double> 
   */
  std::vector<double> get_t() { return t_; }

private:
  Spline sx_ = Spline();
  Spline sy_ = Spline();
  // 媒介変数t(0~1)
  std::vector<double> t_;
  std::vector<double> x_;
  std::vector<double> y_;
};