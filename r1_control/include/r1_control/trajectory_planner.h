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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include "r1_control/accel_designer.h"
#include "r1_control/minimum_jerk.h"
#include "r1_control/spline.h"
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
   * @param theta_wp 角度のwaypoint配列(始点と終点は必須、中間点は任意)
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
    std::vector<double> x_wp, std::vector<double> y_wp, std::vector<double> theta_wp,
    std::vector<double> v_trans_wp, double dt, double v_max, double a_max, double j_max,
    double omega_max)
  {
    // statusを初期化
    status_.clear();

    // パラメータが適切であることを確認
    if (x_wp.size() < 3 || y_wp.size() < 3) {
      RCLCPP_ERROR(
        this->logger_, "Error: The number of waypoints must be 3 or more. x_wp.size(): %zu",
        x_wp.size());
      status_.push_back(FAILURE);
      return status_;
    }
    // xとyのwaypointの数が一致していることを確認
    if (x_wp.size() != y_wp.size()) {
      RCLCPP_ERROR(
        this->logger_,
        "Error: The number of x waypoints and y waypoints must be the same. x_wp.size(): %zu, "
        "y_wp.size(): %zu",
        x_wp.size(), y_wp.size());
      status_.push_back(FAILURE);
      return status_;
    }
    // thetaとv_transのwaypoint数がxyと一致していることを確認
    if (theta_wp.size() != x_wp.size()) {
      RCLCPP_ERROR(
        this->logger_,
        "Error: The number of theta waypoints must match x/y waypoints. theta_wp.size(): %zu, "
        "x_wp.size(): %zu",
        theta_wp.size(), x_wp.size());
      status_.push_back(FAILURE);
      return status_;
    }
    if (v_trans_wp.size() != x_wp.size()) {
      RCLCPP_ERROR(
        this->logger_,
        "Error: The number of velocity waypoints must match x/y waypoints. v_trans_wp.size(): %zu, "
        "x_wp.size(): %zu",
        v_trans_wp.size(), x_wp.size());
      status_.push_back(FAILURE);
      return status_;
    }

    // パラメータの設定
    x_wp_ = x_wp;
    y_wp_ = y_wp;
    theta_wp_ = theta_wp;
    v_trans_wp_ = v_trans_wp;
    theta_wp_index_ = collect_constraint_indices(theta_wp_);
    v_trans_wp_index_ = collect_constraint_indices(v_trans_wp_);

    // v_trans_wpのサイズが適切か確認
    if (v_trans_wp_index_.size() < 2) {
      RCLCPP_ERROR(
        this->logger_,
        "Error: The number of velocity waypoints must be 2 or more (start and end points). ");
      status_.push_back(FAILURE);
      return status_;
    }
    // v_trans_wpの始点と終点が適切であることを確認
    if (v_trans_wp_index_.front() != 0 || v_trans_wp_index_.back() != (int)x_wp.size() - 1) {
      RCLCPP_ERROR(
        this->logger_,
        "Error: The first and last velocity waypoints must be set at the start and end points.");
      status_.push_back(FAILURE);
      return status_;
    }
    // theta_wpのサイズが適切か確認
    if (theta_wp_index_.size() < 2) {
      RCLCPP_ERROR(
        this->logger_,
        "Error: The number of theta waypoints must be 2 or more (start and end points). "
        "theta_wp.size(): %zu",
        theta_wp_index_.size());
      status_.push_back(FAILURE);
      return status_;
    }
    // theta_wpの始点と終点が存在していることを確認
    if (theta_wp_index_.front() != 0 || theta_wp_index_.back() != (int)x_wp.size() - 1) {
      RCLCPP_ERROR(
        this->logger_,
        "Error: The first and last theta waypoints must be set at the start and end points.");
      status_.push_back(FAILURE);
      return status_;
    }
    dt_ = dt;
    v_max_ = v_max;
    a_max_ = a_max;
    j_max_ = j_max;
    // waypointの数
    xy_wp_num_ = (int)x_wp.size();
    v_trans_wp_num_ = (int)v_trans_wp_index_.size();
    theta_wp_num_ = (int)theta_wp_index_.size();
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
    v_segment_dist_.resize((v_trans_wp_num_ - 1));
    // 定速区間として扱う場合に使う補助配列
    v_segment_is_constant_.resize((v_trans_wp_num_ - 1));
    v_segment_const_speed_.resize((v_trans_wp_num_ - 1));
    v_segment_s_start_.resize((v_trans_wp_num_ - 1));
    v_segment_t_start_.resize((v_trans_wp_num_ - 1));
    v_segment_t_end_.resize((v_trans_wp_num_ - 1));
    // 角度の拘束条件の数-1個分にresizeする
    theta_segment_dist_.resize((theta_wp_num_ - 1));
    // 初期化
    dist_all_ = 0.0;
    for (int i = 0; i < (int)v_segment_dist_.size(); i++) {
      v_segment_dist_[i] = 0.0;
      // 定速区間情報もここでリセットしておく。
      v_segment_is_constant_[i] = false;
      v_segment_const_speed_[i] = 0.0;
      v_segment_s_start_[i] = 0.0;
      v_segment_t_start_[i] = 0.0;
      v_segment_t_end_[i] = 0.0;
    }
    for (int i = 0; i < (int)theta_segment_dist_.size(); i++) {
      theta_segment_dist_[i] = 0.0;
    }

    int k = 0, l = 0;
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
        v_segment_dist_[k] += delta_d;
        theta_segment_dist_[l] += delta_d;
        dist_all_ += delta_d;
      }
      // v_segment_dist_のインデックスkを更新
      if (k + 1 < (int)v_segment_dist_.size() && i == v_trans_wp_index_[k + 1]) {
        k++;
      }
      // theta_segment_dist_のインデックスlを更新
      if (l + 1 < (int)theta_segment_dist_.size() && i == theta_wp_index_[l + 1]) {
        l++;
      }
    }

    // 始点位置と始点時刻
    double xs = 0.0;
    double ts = 0.0;

    for (int i = 0; i < v_trans_wp_num_ - 1; i++) {
      const double v_start = v_trans_wp_[v_trans_wp_index_[i]];
      const double v_end = v_trans_wp_[v_trans_wp_index_[i + 1]];
      // 後段のサンプリングで参照できるよう、各速度区間の開始位置と開始時刻を保存する。
      v_segment_s_start_[i] = xs;
      v_segment_t_start_[i] = ts;

      // 元の waypoint 行で隣り合う速度拘束が同じ値なら、その区間は定速区間として扱う。
      if (is_constant_velocity_segment(i)) {
        const double segment_speed = 0.5 * (v_start + v_end);
        if (v_segment_dist_[i] > distance_epsilon_ && segment_speed <= min_constant_speed_) {
          RCLCPP_ERROR(
            this->logger_,
            "Error: Repeated non-positive v_trans cannot form a constant-speed section. "
            "segment=%d, v_start=%f, v_end=%f, dist=%f",
            i, v_start, v_end, v_segment_dist_[i]);
          status_[i] = FAILURE;
          return status_;
        }

        v_segment_is_constant_[i] = true;
        v_segment_const_speed_[i] = segment_speed;
        status_[i] = OK;
        // 定速区間は距離と速度から終了時刻を直接決める。
        if (v_segment_dist_[i] > distance_epsilon_) {
          ts += v_segment_dist_[i] / segment_speed;
        }
      } else {
        // 各区間ごとの速度を計算
        int status = accel_designer_[i].reset(
          j_max_, a_max_, v_max_, v_start, v_end, v_segment_dist_[i], xs, ts);
        status_[i] = status;
        ts = accel_designer_[i].t_end();
      }
      v_segment_t_end_[i] = ts;

      // 次の始点位置、始点時刻の更新
      xs += v_segment_dist_[i];
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
    k = 0;
    ts = 0.0;
    double theta_dist = theta_segment_dist_[0];

    for (int i = 0; i < array_size_; i++) {
      // 時刻を計算
      t_[i] = dt_ * (double)i;
      // 区間ごとのインデックス更新
      if (j < (int)v_segment_t_end_.size() - 1 && t_[i] > v_segment_t_end_[j]) {
        j++;
      }
      if (v_segment_is_constant_[j]) {
        // 定速区間では、区間の経過時間の割合から累積距離を線形に求める。
        // つまり位置は時間に比例して進み、速度は指定値で一定、加速度と躍度は 0 として扱う。
        const double segment_duration = v_segment_t_end_[j] - v_segment_t_start_[j];
        double segment_ratio = 0.0;
        if (segment_duration > distance_epsilon_) {
          segment_ratio = (t_[i] - v_segment_t_start_[j]) / segment_duration;
          segment_ratio = std::clamp(segment_ratio, 0.0, 1.0);
        }
        distance_[i] = v_segment_s_start_[j] + v_segment_dist_[j] * segment_ratio;
        v_trans_[i] = v_segment_const_speed_[j];
        a_trans_[i] = 0.0;
        j_trans_[i] = 0.0;
      } else {
        distance_[i] = accel_designer_[j].x(t_[i]);
        v_trans_[i] = accel_designer_[j].v(t_[i]);
        a_trans_[i] = accel_designer_[j].a(t_[i]);
        j_trans_[i] = accel_designer_[j].j(t_[i]);
      }

      // 走行距離distanceからx,yを逆算

      // 正規化して、媒介変数tを計算
      double t = 0.0;
      if (dist_all_ > distance_epsilon_) {
        t = distance_[i] / dist_all_;
      }
      t = std::clamp(t, 0.0, 1.0);
      // 位置を取得
      auto [x, y] = spline2d_.get_pos(t);
      x_[i] = x;
      y_[i] = y;
      // 曲率を取得
      curvature_[i] = spline2d_.get_curvature(t);

      // thetaの区間が更新されたかを確認。ただし、最後の区間の判定は別途パラメータを指定するため行わない
      bool is_theta_segment_update = false;
      if (k < (int)minimum_jerk_.size() - 1 && distance_[i] > theta_dist) {
        // 次の区間の距離を加算
        theta_dist += theta_segment_dist_[k + 1];
        // インデックスを更新
        k++;
        is_theta_segment_update = true;
      }

      if (is_theta_segment_update) {
        // 区間が更新された場合はパラメータを設定
        // 角度が-piからpiへの移り変わりを考慮するのが面倒なので、始点は0、終点は差分で設定する
        double theta_diff =
          angle_diff(theta_wp_[theta_wp_index_[k]], theta_wp_[theta_wp_index_[k - 1]]);
        minimum_jerk_[k - 1].setParam(0, theta_diff, ts, t_[i]);
        // 次の区間の始点時刻を更新
        ts = t_[i];
      }
    }
    // 最後の区間のパラメータはここで設定
    // 別途設定する理由は積分の誤差でdistance_[i] > theta_distが成立しないことがあるため
    k = (int)minimum_jerk_.size() - 1;
    double theta_diff =
      angle_diff(theta_wp_[theta_wp_index_[k + 1]], theta_wp_[theta_wp_index_[k]]);
    minimum_jerk_[k].setParam(0, theta_diff, ts, t_.back());

    // minimum_jerkを計算
    k = 0;
    int v_status_index = 0;
    for (int i = 0; i < array_size_; i++) {
      if (k < (int)minimum_jerk_.size() - 1 && t_[i] > minimum_jerk_[k].get_tf()) {
        k++;
      }
      if (
        v_status_index < (int)v_segment_t_end_.size() - 1 &&
        t_[i] > v_segment_t_end_[v_status_index]) {
        v_status_index++;
      }
      // 角度を計算、minimum_jerkは始点を0としているので、waypointの角度を足す
      double theta = minimum_jerk_[k].x(t_[i]) + theta_wp_[theta_wp_index_[k]];
      // 角度を-piからpiに正規化
      theta_[i] = angle_normalize(theta);
      omega_[i] = minimum_jerk_[k].v(t_[i]);

      // 最大角速度を超えている場合はwarningを出す
      if ((omega_[i]) > omega_max) {
        RCLCPP_WARN(
          this->logger_,
          "Warning: The angular velocity exceeds the maximum angular velocity at time "
          "%lf s: %lf rad/s",
          t_[i], omega_[i]);
        status_[v_status_index] = WARNING_OMEGA_EXCEEDED;
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
      if (std::isfinite(theta_wp_[i])) {
        fprintf(fp, "%lf", theta_wp_[i]);
      }
      fprintf(fp, ",");
      if (std::isfinite(v_trans_wp_[i])) {
        fprintf(fp, "%lf", v_trans_wp_[i]);
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
  /**
   * @brief 拘束条件として有効な waypoint の添字を抽出する
   *
   * `theta_wp` や `v_trans_wp` は `x_wp`, `y_wp` と同じ長さの配列で保持し、
   * 拘束を置かない waypoint には `inf` などの非有限値を入れる。
   * この関数は配列全体を走査し、`std::isfinite()` が true となる要素の
   * 添字だけを順番通りに集めて返す。
   *
   * 例:
   * `{0.0, inf, inf, 1.57, inf, 3.14}` -> `{0, 3, 5}`
   *
   * @param waypoints 拘束条件付き waypoint 配列
   * @return std::vector<int> 有限値を持つ waypoint の添字配列
   */
  std::vector<int> collect_constraint_indices(std::vector<double> waypoints)
  {
    std::vector<int> indices;
    indices.reserve(waypoints.size());
    for (int i = 0; i < (int)waypoints.size(); i++) {
      if (std::isfinite(waypoints[i])) {
        indices.push_back(i);
      }
    }
    return indices;
  }

  /**
   * @brief 隣接する速度拘束がほぼ同じ値なら、その区間は定速区間として扱う。
   * 
   * @param v_start 
   * @param v_end 
   * @return true 
   * @return false 
   */
  bool is_constant_velocity_segment(int segment_index) const
  {
    const int start_index = v_trans_wp_index_[segment_index];
    const int end_index = v_trans_wp_index_[segment_index + 1];
    if (end_index != start_index + 1) {
      return false;
    }
    const double v_start = v_trans_wp_[start_index];
    const double v_end = v_trans_wp_[end_index];
    return std::isfinite(v_start) && std::isfinite(v_end) &&
           std::abs(v_start - v_end) <= repeated_velocity_tolerance_;
  }

  Spline2D spline2d_;
  std::vector<AccelDesigner> accel_designer_;
  std::vector<MinimumJerk> minimum_jerk_;
  std::vector<double> x_wp_;
  std::vector<double> y_wp_;
  std::vector<double> theta_wp_;
  std::vector<double> v_trans_wp_;
  std::vector<int> theta_wp_index_;
  std::vector<int> v_trans_wp_index_;
  int xy_wp_num_;
  int v_trans_wp_num_;
  int theta_wp_num_;
  double dt_;
  double v_max_;
  double a_max_;
  double j_max_;
  // 分割数
  int segment_num_ = 500;
  // 速度の拘束条件の各区間の距離
  std::vector<double> v_segment_dist_;
  // 連続する同一 v_trans は定速区間として扱う
  std::vector<bool> v_segment_is_constant_;
  // 定速区間として扱うときの区間内速度
  std::vector<double> v_segment_const_speed_;
  // 各速度区間の累積距離開始位置
  std::vector<double> v_segment_s_start_;
  // 各速度区間の開始時刻と終了時刻
  std::vector<double> v_segment_t_start_;
  std::vector<double> v_segment_t_end_;
  // 角度の拘束条件の各区間の距離
  std::vector<double> theta_segment_dist_;
  // 全区間の距離
  double dist_all_;
  // 軌道全体の時間
  double t_all_;
  // 連続する速度拘束を同一値とみなす許容誤差
  double repeated_velocity_tolerance_ = 1e-3;
  // 定速区間として扱える最小速度
  double min_constant_speed_ = 1e-6;
  // 距離・時間のゼロ判定に使う微小値
  double distance_epsilon_ = 1e-9;
  rclcpp::Logger logger_;

public:
  // 計算結果、簡単にアクセスできるようにpublicにする
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
};
