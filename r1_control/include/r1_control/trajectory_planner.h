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
#include <utility>

#include "r1_control/accel_designer.h"
#include "r1_control/minimum_jerk.h"
#include "r1_control/spline.h"
#include "r1_control/util.h"
#include "rclcpp/rclcpp.hpp"

/**
めも
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

class TrajectoryPlanner
{
public:
  // pybind11でstatic変数にアクセスするのは面倒なので、staticにはしない
  const int OK = 0;
  const int WARNING_VELOCITY_NOT_REACHED = -1;
  const int WARNING_OMEGA_EXCEEDED = -2;
  const int FAILURE = -3;
  TrajectoryPlanner() : logger_(rclcpp::get_logger("trajectory_planner")) {}

  /**
   * @brief 与えられたwaypointと拘束条件から軌道を計算する
   * 
   * @param x_wp X座標のwaypoint配列(必須)
   * @param y_wp Y座標のwaypoint配列(必須)
   * @param theta_wp 角度のwaypoint配列(始点と終点は必須、中間点は任意だが、theta_wpを設定するときはv_trans_wpも設定すること)
   * @param v_trans_wp 並進速度のwaypoint配列(始点と終点は必須、中間点は任意)
   * @param dt 制御周期
   * @param v_max 最大速度（絶対値）
   * @param a_max 最大加速度（絶対値）
   * @param j_max 最大躍度（絶対値）
   * @param omega_max 最大角速度（絶対値）、生成した軌道が、最大角速度未満であるかの確認に使用。最大角速度を超えていた場合はwarningを出す。
   * 
   * @return std::vector<int> 各区間の計算状態、大きさはwaypoint数-1
   *  0: 正常終了
   * -1: warning(終端速度が目標速度になっていない)
   * -2: warning(最大角速度を超えている)
   * -3: 失敗
   */
  std::vector<int> calc(
    std::vector<double> x_wp, std::vector<double> y_wp,
    std::vector<std::pair<int, double>> theta_wp, std::vector<std::pair<int, double>> v_trans_wp,
    double dt, double v_max, double a_max, double j_max, double omega_max)
  {
    // statusを初期化
    status_.clear();

    // パラメータが適切であることを確認
    if (x_wp.size() < 3 || y_wp.size() < 3) {
      RCLCPP_ERROR(
        this->logger_, "Error: The number of waypoints must be 3 or more. x_wp.size(): %zu",
        x_wp.size());
      return status_;
    }
    // xとyのwaypointの数が一致していることを確認
    if (x_wp.size() != y_wp.size()) {
      RCLCPP_ERROR(
        this->logger_,
        "Error: The number of x waypoints and y waypoints must be the same. x_wp.size(): %zu, "
        "y_wp.size(): %zu",
        x_wp.size(), y_wp.size());
      return status_;
    }

    // パラメータの設定
    x_wp_ = x_wp;
    y_wp_ = y_wp;
    theta_wp_ = theta_wp;
    v_trans_wp_ = v_trans_wp;
    dt_ = dt;
    v_max_ = v_max;
    a_max_ = a_max;
    j_max_ = j_max;
    // waypointの数
    xy_wp_num_ = (int)x_wp.size();
    v_trans_wp_num_ = (int)v_trans_wp_.size();
    theta_wp_num_ = (int)theta_wp_.size();
    // 各クラスのオブジェクトを生成
    // accel_designerは速度の拘束条件の数-1個分生成する
    accel_designer_.resize(v_trans_wp_num_ - 1);
    // minimum_jerkは角度の拘束条件の数-1個分生成する
    minimum_jerk_.resize(theta_wp_num_ - 1);
    // 各区間分の生成結果の返り値を格納する配列
    status_.resize(xy_wp_num_ - 1);

    // spline 2Dの計算
    spline2d_.calc(x_wp_, y_wp_);
    // 各区間の距離計算
    // 各waypointに対応する媒介変数tを取得
    auto st = spline2d_.get_t();
    // 速度の拘束条件の数-1個分にresizeする
    dist_segment_.resize((v_trans_wp_num_ - 1));
    // 初期化
    dist_all_ = 0.0;
    for (int i = 0; i < (int)dist_segment_.size(); i++) {
      dist_segment_[i] = 0.0;
    }

    int k = 0;
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
        dist_segment_[k] += delta_d;
        dist_all_ += delta_d;
      }
      // dist_segment_のインデックスkを更新
      if (k + 1 < (int)dist_segment_.size() && i == v_trans_wp_[k + 1].first) {
        k++;
      }
    }

    double xs = 0.0;
    double ts = 0.0;

    for (int i = 0; i < v_trans_wp_num_ - 1; i++) {
      // 各区間ごとの速度を計算
      int status = accel_designer_[i].reset(
        j_max_, a_max_, v_max_, v_trans_wp_[i].second, v_trans_wp_[i + 1].second, dist_segment_[i],
        xs, ts);
      status_[i] = status;

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

    // 最小躍度軌道のパラメータを設定
    for (int i = 0; i < theta_wp_num_ - 1; i++) {
      int wp_idx_start = theta_wp_[i].first;
      int wp_idx_end = theta_wp_[i + 1].first;

      double t_start = 0.0;
      double t_end = 0.0;

      // 始点(wp_idx_start)に対応する時刻を探す
      // v_trans_wp_からwp_idx_startと一致するインデックスを探し、その区間の開始時刻を取得
      for (int k = 0; k < v_trans_wp_num_ - 1; k++) {
        if (v_trans_wp_[k].first == wp_idx_start) {
          t_start = accel_designer_[k].t_0();
          break;
        }
      }

      // 終点(wp_idx_end)に対応する時刻を探す
      // v_trans_wp_ から wp_idx_end と一致するインデックスを探す
      // その点は区間の「終わり」なので、一つ前の区間(k-1)の終了時刻を取得する
      for (int k = 1; k < v_trans_wp_num_; k++) {
        if (v_trans_wp_[k].first == wp_idx_end) {
          t_end = accel_designer_[k - 1].t_end();
          break;
        }
      }

      // パラメータを設定
      minimum_jerk_[i].setParam(theta_wp_[i].second, theta_wp_[i + 1].second, t_start, t_end);
    }

    // 区間ごとのインデックス用
    int j = 0;
    k = 0;

    for (int i = 0; i < array_size_; i++) {
      // 時刻を計算
      t_[i] = dt_ * (double)i;
      // 区間ごとのインデックス更新
      while (j < (int)accel_designer_.size() - 1 && t_[i] > accel_designer_[j].t_end()) {
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
      while (k < minimum_jerk_.size() - 1 && t_[i] > minimum_jerk_[k].get_tf()) {
        k++;
      }
      theta_[i] = minimum_jerk_[k].x(t_[i]);
      omega_[i] = minimum_jerk_[k].v(t_[i]);

      // 最大角速度を超えている場合はwarningを出す
      if ((omega_[i]) > omega_max) {
        RCLCPP_WARN(
          this->logger_,
          "Warning: The angular velocity exceeds the maximum angular velocity at time "
          "%lf s: %lf rad/s",
          t_[i], omega_[i]);
        status_[j] = WARNING_OMEGA_EXCEEDED;
      }
    }

    return status_;
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
    for (int i = 0; i < xy_wp_num_; i++) {
      fprintf(fp, "%lf,%lf,", x_wp_[i], y_wp_[i]);
      for (int j = 0; j < theta_wp_num_; j++) {
        if (theta_wp_[j].first == i) {
          fprintf(fp, "%lf", theta_wp_[j].second);
          break;
        }
      }
      fprintf(fp, ",");
      for (int j = 0; j < v_trans_wp_num_; j++) {
        if (v_trans_wp_[j].first == i) {
          fprintf(fp, "%lf", v_trans_wp_[j].second);
          break;
        }
      }
      fprintf(fp, "\n");
    }
  }

  /**
   * @brief 軌道を取得する
   * 返り値は
   * [0]: 各区間の計算状態の配列、大きさはwaypoint数-1
   * [1]: 時刻tの配列
   * [2]: x座標の配列
   * [3]: y座標の配列
   * [4]: 角度thetaの配列
   * [5]: 走行距離distanceの配列
   * [6]: 並進速度v_transの配列
   * [7]: 並進加速度a_transの配列
   * [8]: 並進躍度j_transの配列
   * [9]: 角速度omegaの配列
   * [10]: 曲率curvatureの配列
   * 
   * @return std::tuple<
   * std::vector<int>, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
   * std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
   * std::vector<double>, std::vector<double>> 
   */
  std::tuple<
    std::vector<int>, std::vector<double>, std::vector<double>, std::vector<double>,
    std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
    std::vector<double>, std::vector<double>, std::vector<double>>
  get_trajectory()
  {
    return std::make_tuple(
      status_, t_, x_, y_, theta_, distance_, v_trans_, a_trans_, j_trans_, omega_, curvature_);
  }

private:
  Spline2D spline2d_;
  std::vector<AccelDesigner> accel_designer_;
  std::vector<MinimumJerk> minimum_jerk_;
  std::vector<double> x_wp_;
  std::vector<double> y_wp_;
  // first: index, second: value
  std::vector<std::pair<int, double>> theta_wp_;
  // first: index, second: value
  std::vector<std::pair<int, double>> v_trans_wp_;
  int xy_wp_num_;
  int v_trans_wp_num_;
  int theta_wp_num_;
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
  // 各区間の計算状態
  std::vector<int> status_;
  rclcpp::Logger logger_;
};
