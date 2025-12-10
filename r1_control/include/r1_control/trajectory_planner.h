/**
 * @file trajectory_planner.h
 * @author Yudai Yamaguchi
 * @brief 経路計画をするクラス
 * @version 0.1
 * @date 2025-12-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <cstdio>

#include "r1_control/accel_designer.h"
#include "r1_control/minimum_jerk.h"
#include "r1_control/spline.h"
#include "r1_control/util.h"
#include "rclcpp/rclcpp.hpp"

/**
- 必要な情報
  - waypoint(x, y, theta, v_trans(任意))
  - 制御周期
  - v_max
  - a_max
  - j_max
  
 - waypoint(x,y)をspline 2Dに入力
 - 各区間ごとの距離と全区間の距離を計算
 - 制約条件に基づいて、AccelDesignerを使って制御周期ごとの速度を計算し、変数に格納
 - 取得した拘束条件より、5次最小躍度軌道を用いて、ロボットの角度を計算
 - 計算結果をプロットする
 * 
 */

// TODO: どこかにアルゴリズムの概要を書く
// TODO: waypointの速度や角度が変化しない場合、いい感じにする

class TrajectoryPlanner
{
public:
  TrajectoryPlanner() {}

  /**
   * @brief 与えられたwaypointと拘束条件から軌道を計算する
   * 
   * @param x_wp X座標のwaypoint配列
   * @param y_wp Y座標のwaypoint配列
   * @param theta_wp 角度のwaypoint配列
   * @param v_trans_wp 並進速度のwaypoint配列
   * @param dt 制御周期
   * @param v_max 最大速度（絶対値）
   * @param a_max 最大加速度（絶対値）
   * @param j_max 最大躍度（絶対値）
   */
  void calc(
    std::vector<double> x_wp, std::vector<double> y_wp, std::vector<double> theta_wp,
    std::vector<double> v_trans_wp, double dt, double v_max, double a_max, double j_max)
  {
    // パラメータの設定
    x_wp_ = x_wp;
    y_wp_ = y_wp;
    theta_wp_ = theta_wp;
    v_trans_wp_ = v_trans_wp;
    dt_ = dt;
    v_max_ = v_max;
    a_max_ = a_max;
    j_max_ = j_max;
    wp_num_ = (int)x_wp.size();
    // 各クラスのオブジェクトを生成
    accel_designer_.resize(wp_num_ - 1);
    minimum_jerk_.resize(wp_num_ - 1);
    // spline 2Dの計算
    spline2d_.calc(x_wp_, y_wp_);
    // 各区間の距離計算
    // 各waypointに対応する媒介変数tを取得
    auto st = spline2d_.get_t();
    // resize&初期化
    dist_segment_.resize((wp_num_ - 1));
    dist_all_ = 0.0;
    for (int i = 0; i < (int)dist_segment_.size(); i++) {
      dist_segment_[i] = 0.0;
    }

    for (int i = 1; i < (int)st.size(); i++) {
      for (int j = 1; j <= segment_num_; j++) {
        // 媒介変数をsegment_num_個だけ、分割して計算
        double ts = st[i - 1] + (st[i] - st[i - 1]) * (double)(j - 1) / (double)segment_num_;
        double te = st[i - 1] + (st[i] - st[i - 1]) * (double)j / (double)segment_num_;
        // スプライン上の座標を取得
        auto [sx, sy] = spline2d_.get_pos(ts);
        auto [ex, ey] = spline2d_.get_pos(te);
        // 微小距離を計算
        double delta_d = hypot(ex - sx, ey - sy);
        // 微小距離を積分
        dist_segment_[i - 1] += delta_d;
        dist_all_ += delta_d;
      }
    }

    double xs = 0.0;
    double ts = 0.0;

    for (int i = 0; i < wp_num_ - 1; i++) {
      // 各区間ごとの速度を計算
      accel_designer_[i].reset(
        j_max_, a_max_, v_max_, v_trans_wp_[i], v_trans_wp_[i + 1], dist_segment_[i], xs, ts);

      // 次の始点位置、始点時刻の更新
      xs += dist_segment_[i];
      ts = accel_designer_[i].t_end();
    }

    // 軌道の終了時刻/分割数の配列を作成
    t_all_ = ts;

    array_size_ = (int)(t_all_ / dt_) + 1;
    t_.resize(array_size_);
    x_.resize(array_size_);
    y_.resize(array_size_);
    distance_.resize(array_size_);
    theta_.resize(array_size_);
    v_trans_.resize(array_size_);
    a_trans_.resize(array_size_);
    j_trans_.resize(array_size_);
    omega_.resize(array_size_);
    curvature_.resize(array_size_);

    // 区間ごとのインデックス用
    int j = 0;

    // 最小躍度軌道のパラメータを設定
    for (int i = 0; i < wp_num_ - 1; i++) {
      minimum_jerk_[i].setParam(
        theta_wp_[i], theta_wp_[i + 1], accel_designer_[i].t_0(), accel_designer_[i].t_end());
    }

    for (int i = 0; i < array_size_; i++) {
      // 時刻を計算
      t_[i] = dt_ * (double)i;
      // 区間ごとのインデックス更新
      while (j < wp_num_ - 1 && t_[i] > accel_designer_[j].t_end()) {
        j++;
      }
      distance_[i] = accel_designer_[j].x(t_[i]);
      v_trans_[i] = accel_designer_[j].v(t_[i]);
      a_trans_[i] = accel_designer_[j].a(t_[i]);
      j_trans_[i] = accel_designer_[j].j(t_[i]);

      // 走行距離distanceからx,yを逆算

      // 正規化して、媒介変数tを計算
      double t = distance_[i] / dist_all_;
      // 位置を取得
      auto [x, y] = spline2d_.get_pos(t);
      x_[i] = x;
      y_[i] = y;
      // 曲率を取得
      curvature_[i] = spline2d_.get_curvature(t);
      // 最小躍度軌道を用いて、角度と角速度を計算
      theta_[i] = minimum_jerk_[j].x(t_[i]);
      omega_[i] = minimum_jerk_[j].v(t_[i]);
    }
  }

  /**
     * @brief 生成した軌道をCSV形式で出力する
     * 
     * @param fp 
     */
  void print_csv_trajectory(FILE * fp)
  {
    for (int i = 0; i < array_size_; i++) {
      fprintf(
        fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", t_[i], x_[i], y_[i], theta_[i],
        distance_[i], v_trans_[i], a_trans_[i], j_trans_[i], omega_[i], curvature_[i]);
    }
  }

  /**
     * @brief 与えられたwaypointをCSV形式で出力する
     * 
     * @param fp 
     */
  void print_csv_waypoint(FILE * fp)
  {
    for (int i = 0; i < wp_num_; i++) {
      fprintf(fp, "%lf,%lf,%lf,%lf\n", x_wp_[i], y_wp_[i], theta_wp_[i], v_trans_wp_[i]);
    }
  }

  /**
   * @brief 軌道を取得する
   * 返り値は
   * [0]: 時刻tの配列
   * [1]: x座標の配列
   * [2]: y座標の配列
   * [3]: 角度thetaの配列
   * [4]: 走行距離distanceの配列
   * [5]: 並進速度v_transの配列
   * [6]: 並進加速度a_transの配列
   * [7]: 並進躍度j_transの配列
   * [8]: 角速度omegaの配列
   * [9]: 曲率curvatureの配列
   * 
   * @return std::tuple<
   * std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
   * std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
   * std::vector<double>, std::vector<double>> 
   */
  std::tuple<
    std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
    std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
    std::vector<double>, std::vector<double>>
  get_trajectory()
  {
    return std::make_tuple(
      t_, x_, y_, theta_, distance_, v_trans_, a_trans_, j_trans_, omega_, curvature_);
  }

private:
  Spline2D spline2d_;
  std::vector<AccelDesigner> accel_designer_;
  std::vector<MinimumJerk> minimum_jerk_;
  std::vector<double> x_wp_;
  std::vector<double> y_wp_;
  std::vector<double> theta_wp_;
  std::vector<double> v_trans_wp_;
  int wp_num_;
  double dt_;
  double v_max_;
  double a_max_;
  double j_max_;
  // 分割数
  int segment_num_ = 500;
  // 各区間の距離
  std::vector<double> dist_segment_;
  double dist_all_;
  double t_all_;
  // 計算結果
  std::vector<double> t_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> distance_;
  std::vector<double> theta_;
  std::vector<double> v_trans_;
  std::vector<double> a_trans_;
  std::vector<double> j_trans_;
  std::vector<double> omega_;
  std::vector<double> curvature_;
  int array_size_;
};
